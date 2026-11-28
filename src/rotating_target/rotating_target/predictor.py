"""Predict where the rotating handle will be.

Reads the handle position on /target/handle_true (later: from the camera),
turns it into an angle around the turntable centre, estimates how fast
that angle changes, and predicts the angle `horizon` seconds ahead.

It also checks itself: when the predicted moment arrives, it compares the
prediction with where the handle actually was, and reports the error.

Methods (parameter `method`):
  reactive  - no prediction; assume the handle stays where it was seen
  last_two  - speed from the two most recent measurements
  average   - speed from a straight-line fit through the last `window`
              measurements (smooths out noise)
  kalman    - Kalman filter tracking angle and speed together

Experiment settings (all optional):
  noise_std   - add random error (m) to each measured x and y, to imitate a
                camera. The error check always uses the clean position.
  delay       - make each measurement arrive this many seconds late, to
                imitate camera and processing delay.
  compensate  - true: predict from the measurement's own time, so the delay
                is corrected. false: treat a late measurement as current.
  log_file    - CSV file; one row per checked prediction.
  run_time    - stop after this many seconds of logged checks (0 = never).
  warmup      - seconds after the first prediction before logging starts,
                so every method has a full history.
  spin_period - the target's set spin period (s). Only written to the log,
                never used for prediction.
"""

import csv
import math
import os
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray


def wrap(a):
    """Bring an angle into the range -pi..pi."""
    return math.atan2(math.sin(a), math.cos(a))


class Unwrapper:
    """Turns atan2 angles into a smoothly increasing angle (see Part 3)."""

    def __init__(self):
        self.last_raw = None
        self.turns = 0.0

    def __call__(self, raw):
        if self.last_raw is not None:
            jump = raw - self.last_raw
            if jump < -math.pi:
                self.turns += 2.0 * math.pi
            elif jump > math.pi:
                self.turns -= 2.0 * math.pi
        self.last_raw = raw
        return raw + self.turns


class Predictor(Node):

    def __init__(self):
        super().__init__('predictor')

        self.declare_parameter('method', 'average')
        self.declare_parameter('horizon', 1.0)      # seconds ahead
        self.declare_parameter('window', 20)        # samples for 'average'
        self.declare_parameter('center_x', 0.5)     # turntable centre (m)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('kf_q', 0.01)        # Kalman: how much speed may drift
        self.declare_parameter('kf_r', -1.0)        # Kalman: angle noise (rad^2); -1 = from noise_std
        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('delay', 0.0)
        self.declare_parameter('compensate', True)
        self.declare_parameter('log_file', '')
        self.declare_parameter('run_time', 0.0)
        self.declare_parameter('warmup', 5.0)
        self.declare_parameter('spin_period', 0.0)
        self.declare_parameter('seed', 1)

        self.method = self.get_parameter('method').value
        self.horizon = float(self.get_parameter('horizon').value)
        self.window = int(self.get_parameter('window').value)
        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)
        self.q = float(self.get_parameter('kf_q').value)
        self.noise = float(self.get_parameter('noise_std').value)
        self.delay = float(self.get_parameter('delay').value)
        self.compensate = bool(self.get_parameter('compensate').value)
        self.run_time = float(self.get_parameter('run_time').value)
        self.warmup = float(self.get_parameter('warmup').value)
        self.spin_period = float(self.get_parameter('spin_period').value)
        random.seed(int(self.get_parameter('seed').value))

        # Kalman measurement noise: if not set, use the angle error that
        # noise_std causes at the handle radius (0.13 m), with a small floor.
        r = float(self.get_parameter('kf_r').value)
        self.r = r if r > 0 else max((self.noise / 0.13) ** 2, 1e-6)

        if self.method not in ('reactive', 'last_two', 'average', 'kalman'):
            raise ValueError(
                'method must be reactive, last_two, average or kalman')

        # What the predictor sees (possibly noisy and late)
        self.history = []             # (measurement time, unwrapped angle)
        self.unwrap_meas = Unwrapper()
        self.radius = None
        self.z = 0.0
        self.inbox = []               # delayed measurements not yet delivered

        # The clean truth, used only to check predictions
        self.truth = []               # (time, unwrapped angle)
        self.unwrap_true = Unwrapper()
        self.true_radius = None

        # Kalman filter state: angle, speed, and their uncertainty P
        self.kf_x = None
        self.kf_P = None
        self.kf_t = None

        # Predictions waiting to be checked:
        # (time made, target time, predicted angle, estimated speed)
        self.pending = []
        self.errors = []
        self.last_report = None
        self.first_pred_t = None
        self.first_log_t = None
        self.done = False

        self.log = None
        log_file = self.get_parameter('log_file').value
        if log_file:
            folder = os.path.dirname(log_file)
            if folder:
                os.makedirs(folder, exist_ok=True)
            self.log_fh = open(log_file, 'w', newline='')
            self.log = csv.writer(self.log_fh)
            self.log.writerow([
                'method', 'spin_period', 'horizon', 'noise_std', 'delay',
                'compensate', 't_made', 't_target', 'pred_angle',
                'true_angle', 'error_m', 'speed_est'])

        self.sub = self.create_subscription(
            PoseStamped, '/target/handle_true', self.on_truth, 10)
        self.pred_pub = self.create_publisher(
            PoseStamped, '/target/handle_predicted', 10)
        self.err_pub = self.create_publisher(
            Float64, '/target/prediction_error', 10)
        # Current estimate for the capture program (Stage B):
        # [measurement time, angle (rad, unwrapped), speed (rad/s), time now]
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/target/handle_state', 10)

        self.get_logger().info(
            f'Predictor: method={self.method}, horizon={self.horizon} s, '
            f'noise={self.noise * 1000:.1f} mm, delay={self.delay} s, '
            f'compensate={self.compensate}')

    # ---------- 0. clean truth in, noisy and late measurement out ----------

    def on_truth(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y = msg.pose.position.x, msg.pose.position.y
        self.z = msg.pose.position.z

        if self.truth and t <= self.truth[-1][0]:
            return  # out-of-order or repeated message
        dx, dy = x - self.cx, y - self.cy
        r = math.hypot(dx, dy)
        self.true_radius = r if self.true_radius is None else self.true_radius
        self.truth.append((t, self.unwrap_true(math.atan2(dy, dx))))
        self.truth = self.truth[-600:]

        # Imitate the camera: noise on the position, then a delay
        if self.noise > 0:
            x += random.gauss(0.0, self.noise)
            y += random.gauss(0.0, self.noise)
        self.inbox.append((t, x, y))

        # Deliver every measurement whose delay has passed ("now" = t)
        while self.inbox and self.inbox[0][0] + self.delay <= t + 1e-9:
            tm, xm, ym = self.inbox.pop(0)
            self.on_measurement(tm, xm, ym, now=t)

        self.check_predictions(t)

    # ---------- 1. measurement -> angle ----------

    def on_measurement(self, t, x, y, now):
        dx, dy = x - self.cx, y - self.cy
        r = math.hypot(dx, dy)
        self.radius = r if self.radius is None else 0.9 * self.radius + 0.1 * r

        # 2. Unwrap, so the angle keeps increasing smoothly
        theta = self.unwrap_meas(math.atan2(dy, dx))

        if self.history and t <= self.history[-1][0]:
            return
        self.history.append((t, theta))
        self.history = self.history[-max(self.window, 200):]

        # 3. Estimate angle and speed at the measurement time
        theta_meas, omega = self.estimate(t, theta)
        if theta_meas is None:
            return

        # 4. Predict. The prediction is made "now" (when the measurement
        # arrives) for horizon seconds later. With compensation the
        # extrapolation also covers the delay; without, the late
        # measurement is treated as if it were current.
        t_future = now + self.horizon
        ahead = (t_future - t) if self.compensate else self.horizon
        theta_future = theta_meas + omega * ahead

        self.publish_prediction(t_future, theta_future)
        # For the capture program: with delay correction it gets the time the
        # measurement was taken, so it can allow for the measurement's age;
        # without, the late measurement is labelled as current (Stage D).
        t_label = t if self.compensate else now
        self.state_pub.publish(
            Float64MultiArray(data=[t_label, theta_meas, omega, now]))
        self.pending.append((now, t_future, theta_future, omega))
        if self.first_pred_t is None:
            self.first_pred_t = now

    # ---------- 3. the four methods ----------

    def estimate(self, t, theta):
        if self.method == 'reactive':
            return theta, 0.0

        if self.method == 'last_two':
            if len(self.history) < 2:
                return None, None
            (t0, a0), (t1, a1) = self.history[-2], self.history[-1]
            return a1, (a1 - a0) / (t1 - t0)

        if self.method == 'average':
            pts = self.history[-self.window:]
            if len(pts) < 3:
                return None, None
            # Least-squares straight line: angle = a + omega * t
            n = len(pts)
            mt = sum(p[0] for p in pts) / n
            ma = sum(p[1] for p in pts) / n
            stt = sum((p[0] - mt) ** 2 for p in pts)
            sta = sum((p[0] - mt) * (p[1] - ma) for p in pts)
            omega = sta / stt if stt > 0 else 0.0
            return ma + omega * (t - mt), omega

        return self.kalman(t, theta)

    def kalman(self, t, theta):
        if self.kf_x is None:
            self.kf_x = [theta, 0.0]
            self.kf_P = [[1.0, 0.0], [0.0, 10.0]]
            self.kf_t = t
            return None, None

        dt = t - self.kf_t
        self.kf_t = t
        a, w = self.kf_x
        P = self.kf_P

        # Predict: angle moves on by speed * dt; speed may drift a little
        a = a + w * dt
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1]
        p01 = P[0][1] + dt * P[1][1]
        p10 = P[1][0] + dt * P[1][1]
        p11 = P[1][1]
        p00 += self.q * dt ** 3 / 3.0
        p01 += self.q * dt ** 2 / 2.0
        p10 += self.q * dt ** 2 / 2.0
        p11 += self.q * dt

        # Update: blend in the new measurement, weighted by how sure we are
        s = p00 + self.r
        k0 = p00 / s
        k1 = p10 / s
        innov = theta - a
        a = a + k0 * innov
        w = w + k1 * innov
        self.kf_P = [[(1 - k0) * p00, (1 - k0) * p01],
                     [p10 - k1 * p00, p11 - k1 * p01]]
        self.kf_x = [a, w]
        return a, w

    # ---------- publishing and self-check ----------

    def publish_prediction(self, t_future, theta):
        out = PoseStamped()
        out.header.frame_id = 'world'
        out.header.stamp.sec = int(t_future)
        out.header.stamp.nanosec = int((t_future - int(t_future)) * 1e9)
        out.pose.position.x = self.cx + self.radius * math.cos(theta)
        out.pose.position.y = self.cy + self.radius * math.sin(theta)
        out.pose.position.z = self.z
        out.pose.orientation.z = math.sin(theta / 2.0)
        out.pose.orientation.w = math.cos(theta / 2.0)
        self.pred_pub.publish(out)

    def true_angle_at(self, t):
        """Actual angle at time t, interpolated between clean positions."""
        h = self.truth
        for (t0, a0), (t1, a1) in zip(h, h[1:]):
            if t0 <= t <= t1:
                return a0 + (a1 - a0) * (t - t0) / (t1 - t0)
        return None

    def check_predictions(self, now):
        still_waiting = []
        for t_made, t_target, theta_pred, omega in self.pending:
            if t_target > now:
                still_waiting.append((t_made, t_target, theta_pred, omega))
                continue
            actual = self.true_angle_at(t_target)
            if actual is None:
                continue
            # Distance between predicted and actual point on the circle
            err = 2.0 * self.true_radius * abs(
                math.sin(wrap(theta_pred - actual) / 2.0))
            self.err_pub.publish(Float64(data=err))

            if t_made < self.first_pred_t + self.warmup:
                continue  # still warming up: not logged or reported
            if self.first_log_t is None:
                self.first_log_t = now
            self.errors.append(err)
            if self.log is not None:
                self.log.writerow([
                    self.method, self.spin_period, self.horizon, self.noise,
                    self.delay, int(self.compensate), f'{t_made:.3f}',
                    f'{t_target:.3f}', f'{theta_pred:.6f}', f'{actual:.6f}',
                    f'{err:.6f}', f'{omega:.6f}'])
                self.log_fh.flush()
        self.pending = still_waiting

        if self.last_report is None:
            self.last_report = now
        if now - self.last_report >= 2.0 and self.errors:
            mean = sum(self.errors) / len(self.errors)
            worst = max(self.errors)
            self.get_logger().info(
                f'{self.method}: {self.horizon:.1f} s ahead, error mean '
                f'{mean * 1000:.1f} mm, max {worst * 1000:.1f} mm '
                f'({len(self.errors)} checks); speed '
                f'{self.estimate_speed_for_log():.3f} rad/s')
            self.errors = []
            self.last_report = now

        if (self.run_time > 0 and self.first_log_t is not None
                and now - self.first_log_t >= self.run_time):
            self.done = True

    def estimate_speed_for_log(self):
        if len(self.history) < 2:
            return 0.0
        (t0, a0), (t1, a1) = self.history[0], self.history[-1]
        return (a1 - a0) / (t1 - t0)

    def close(self):
        if self.log is not None:
            self.log_fh.close()


def main(args=None):
    rclpy.init(args=args)
    node = Predictor()
    try:
        # Keep handling messages until Ctrl+C, or until run_time is reached
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
