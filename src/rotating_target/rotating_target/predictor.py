""Predict where the rotating handle will be.

Reads the handle position on /target/handle_true (later: from the camera),
turns it into an angle around the turntable centre, estimates how fast
that angle changes, and predicts the angle ahead.

It also checks itself: when the predicted moment arrives, it compares the
prediction with where the handle actually was, and reports the error.

Methods (parameter `method`):
  reactive  - no prediction; assume the handle stays where it was seen
  last_two  - speed from the two most recent measurements
  average   - speed from a straight-line fit through the last `window`
              measurements (smooths out noise)
  kalman    - Kalman filter tracking angle and speed together
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


def wrap(a):
    """Bring an angle into the range -pi..pi."""
    return math.atan2(math.sin(a), math.cos(a))


class Predictor(Node):

    def __init__(self):
        super().__init__('predictor')

        self.declare_parameter('method', 'average')
        self.declare_parameter('horizon', 1.0)      # seconds ahead
        self.declare_parameter('window', 20)        # samples for 'average'
        self.declare_parameter('center_x', 0.5)     # turntable centre (m)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('kf_q', 0.01)        # Kalman: how much speed may drift
        self.declare_parameter('kf_r', 1e-4)        # Kalman: measurement noise (rad^2)

        self.method = self.get_parameter('method').value
        self.horizon = float(self.get_parameter('horizon').value)
        self.window = int(self.get_parameter('window').value)
        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)
        self.q = float(self.get_parameter('kf_q').value)
        self.r = float(self.get_parameter('kf_r').value)

        if self.method not in ('reactive', 'last_two', 'average', 'kalman'):
            raise ValueError(
                'method must be reactive, last_two, average or kalman')

        # Measurement history: (time, unwrapped angle), oldest first
        self.history = []
        self.last_raw = None      # last angle before unwrapping
        self.turns = 0.0          # whole turns added by unwrapping
        self.radius = None        # handle distance from centre (m)
        self.z = 0.0

        # Kalman filter state: angle, speed, and their uncertainty P
        self.kf_x = None
        self.kf_P = None
        self.kf_t = None

        # Predictions waiting to be checked: (target time, predicted angle)
        self.pending = []
        self.errors = []          # position errors (m) since last report
        self.last_report = None

        self.sub = self.create_subscription(
            PoseStamped, '/target/handle_true', self.on_measurement, 10)
        self.pred_pub = self.create_publisher(
            PoseStamped, '/target/handle_predicted', 10)
        self.err_pub = self.create_publisher(
            Float64, '/target/prediction_error', 10)

        self.get_logger().info(
            f'Predictor: method={self.method}, horizon={self.horizon} s')

    # ---------- 1. measurement -> angle ----------

    def on_measurement(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dx = msg.pose.position.x - self.cx
        dy = msg.pose.position.y - self.cy
        self.z = msg.pose.position.z

        r = math.hypot(dx, dy)
        self.radius = r if self.radius is None else 0.9 * self.radius + 0.1 * r

        raw = math.atan2(dy, dx)
        # 2. Unwrap: when the angle jumps from +pi to -pi (or back),
        # count a whole turn so the angle keeps increasing smoothly.
        if self.last_raw is not None:
            jump = raw - self.last_raw
            if jump < -math.pi:
                self.turns += 2.0 * math.pi
            elif jump > math.pi:
                self.turns -= 2.0 * math.pi
        self.last_raw = raw
        theta = raw + self.turns

        if self.history and t <= self.history[-1][0]:
            return  # out-of-order or repeated message
        self.history.append((t, theta))
        self.history = self.history[-max(self.window, 200):]

        self.check_predictions()

        # 3. Estimate angle and speed now, with the chosen method
        theta_now, omega = self.estimate(t, theta)
        if theta_now is None:
            return

        # 4. Predict ahead and 5. convert back to a position
        t_future = t + self.horizon
        theta_future = theta_now + omega * self.horizon
        self.publish_prediction(t_future, theta_future)
        self.pending.append((t_future, theta_future))

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
        """Actual angle at time t, interpolated between measurements."""
        h = self.history
        for (t0, a0), (t1, a1) in zip(h, h[1:]):
            if t0 <= t <= t1:
                return a0 + (a1 - a0) * (t - t0) / (t1 - t0)
        return None

    def check_predictions(self):
        now = self.history[-1][0]
        still_waiting = []
        for t_target, theta_pred in self.pending:
            if t_target > now:
                still_waiting.append((t_target, theta_pred))
                continue
            actual = self.true_angle_at(t_target)
            if actual is None:
                continue
            # Distance between predicted and actual point on the circle
            err = 2.0 * self.radius * abs(math.sin(wrap(theta_pred - actual) / 2.0))
            self.errors.append(err)
            self.err_pub.publish(Float64(data=err))
        self.pending = still_waiting

        if self.last_report is None:
            self.last_report = now
        if now - self.last_report >= 2.0 and self.errors:
            mean = sum(self.errors) / len(self.errors)
            worst = max(self.errors)
            omega = self.estimate_speed_for_log()
            self.get_logger().info(
                f'{self.method}: {self.horizon:.1f} s ahead, error mean '
                f'{mean * 1000:.1f} mm, max {worst * 1000:.1f} mm '
                f'({len(self.errors)} checks); speed {omega:.3f} rad/s')
            self.errors = []
            self.last_report = now

    def estimate_speed_for_log(self):
        if len(self.history) < 2:
            return 0.0
        (t0, a0), (t1, a1) = self.history[0], self.history[-1]
        return (a1 - a0) / (t1 - t0)


def main(args=None):
    rclpy.init(args=args)
    node = Predictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
