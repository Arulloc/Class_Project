#!/usr/bin/env python3
"""capture.py: Stage B - try to catch the spinning handle with the arm.

How one trial works (report section 5.7):
  1. The arm waits at a fixed HOVER pose, 10 cm above a fixed MEETING POINT on
     the handle's circle (the point nearest the arm), gripper pointing down
     with its fingers opening along the handle's direction of travel.
  2. From the predictor's latest angle and speed estimates it works out when
     the handle will next reach the meeting point (reactive: it waits until
     the handle is actually there).
  3. It sends the arm a short descent (hover -> grasp, `descent` seconds) that
     is scheduled to START at (arrival time - descent), so the gripper reaches
     the meeting point at the predicted arrival time.
  4. At that moment it measures where the gripper's grasp point is (from the
     arm's own joint angles) and where the handle really is (ground truth),
     and logs the error: along the motion (timing), across it (aim), vertical.
  5. Back up to hover; next trial.

The two poses are found once, at start-up, by a small inverse-kinematics
solver that reads the arm's own model (robot_description), so no MoveIt is
needed. Everything runs in Gazebo simulation time.

Run (Gazebo + arm already running, target spinning, predictor running):
  python3 scripts/capture.py --ros-args -p use_sim_time:=true \
      -p method:=kalman -p spin_period:=10.0 -p trials:=10 \
      -p log_file:=results/stageB/test.csv
"""

import csv
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
EE_LINK = 'end_effector_link'
TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------- kinematics

def rot_rpy(r, p, y):
    """URDF roll-pitch-yaw to a rotation matrix (R = Rz(y) Ry(p) Rx(r))."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    """Rotation by angle q about a unit axis (Rodrigues' formula)."""
    x, y, z = axis
    c, s, C = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def transform(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class Chain:
    """The joints from the model's root link to `tip`, read from the URDF."""

    def __init__(self, urdf_xml, tip):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall('joint'):
            by_child[j.find('child').get('link')] = j
        steps, link = [], tip
        while link in by_child:          # walk from the tip up to the root
            j = by_child[link]
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None
                                      else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None
                                      else '0 0 0').split()]
            a = j.find('axis')
            axis = [float(v) for v in (a.get('xyz') if a is not None
                                       else '1 0 0').split()]
            lim = j.find('limit')
            lo = hi = None
            if j.get('type') == 'revolute' and lim is not None:
                lo, hi = float(lim.get('lower')), float(lim.get('upper'))
            steps.append(dict(name=j.get('name'), type=j.get('type'),
                              origin=transform(rot_rpy(*rpy), xyz),
                              axis=np.array(axis) / np.linalg.norm(axis),
                              lo=lo, hi=hi))
            link = j.find('parent').get('link')
        self.steps = list(reversed(steps))
        self.root = link
        movable = [s['name'] for s in self.steps if s['type'] != 'fixed']
        if movable != ARM_JOINTS:
            raise RuntimeError(f'unexpected joints to {tip}: {movable}')

    def fk(self, q):
        """4x4 pose of the tip in the root frame for joint angles q."""
        T, i = np.eye(4), 0
        for s in self.steps:
            T = T @ s['origin']
            if s['type'] != 'fixed':
                T = T @ transform(rot_axis(s['axis'], q[i]), [0, 0, 0])
                i += 1
        return T

    def limits(self):
        return [(s['lo'], s['hi']) for s in self.steps if s['type'] != 'fixed']


def pose_error(T, T_goal):
    """6-vector: position error and rotation error (axis * angle)."""
    dp = T_goal[:3, 3] - T[:3, 3]
    R = T_goal[:3, :3] @ T[:3, :3].T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-9:
        dr = np.zeros(3)
    else:
        dr = angle / (2 * math.sin(angle)) * np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return np.concatenate([dp, dr])


def solve_ik(chain, T_goal, seeds, iters=300):
    """Damped least squares from several starting guesses. Returns the joint
    angles closest to the first seed among those that reach the goal."""
    lims = chain.limits()
    best = None
    for seed in seeds:
        q = np.array(seed, dtype=float)
        for _ in range(iters):
            e = pose_error(chain.fk(q), T_goal)
            if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                break
            J = np.zeros((6, 6))
            for k in range(6):          # numerical Jacobian
                dq = np.zeros(6)
                dq[k] = 1e-6
                J[:, k] = (pose_error(chain.fk(q), T_goal)
                           - pose_error(chain.fk(q + dq), T_goal)) / 1e-6
            lam = 1e-3
            q = q + J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
            for k, (lo, hi) in enumerate(lims):
                if lo is not None:
                    q[k] = min(max(q[k], lo), hi)
        e = pose_error(chain.fk(q), T_goal)
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            dist = np.linalg.norm(q - np.array(seeds[0]))
            if best is None or dist < best[0]:
                best = (dist, q)
    return None if best is None else best[1]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- the node

class Capture(Node):

    def __init__(self):
        super().__init__('capture')
        P = self.declare_parameter
        P('method', 'kalman')            # only for the log; set on the predictor
        P('spin_period', 0.0)            # only for the log
        P('trials', 10)
        P('log_file', '')
        P('center_x', 0.5)               # turntable centre (m), as predictor
        P('center_y', 0.0)
        P('handle_radius', 0.13)
        P('handle_z', 0.20)              # height of the grip bar's middle
        P('meet_angle', math.pi)         # meeting point: angle around centre
        P('hover_height', 0.10)          # hover this far above the grip point
        P('descent', 0.5)                # seconds from hover to grasp
        P('lead_min', 0.5)               # commit when arrival is between
        P('lead_max', 1.0)               #   descent+lead_min and descent+lead_max ahead
        P('hold', 0.4)                   # stay down this long, then rise
        P('tcp_offset', 0.12)            # grasp point beyond end_effector_link (m)
        P('finger_axis', 'y')            # end-effector axis the fingers open along
        P('tolerance', 0.036)            # capture tolerance (m), report 6.2
        g = lambda n: self.get_parameter(n).value
        self.method = g('method')
        self.trials = int(g('trials'))
        self.cx, self.cy = float(g('center_x')), float(g('center_y'))
        self.r = float(g('handle_radius'))
        self.hz = float(g('handle_z'))
        self.theta_m = float(g('meet_angle'))
        self.descent = float(g('descent'))
        self.lead_min, self.lead_max = float(g('lead_min')), float(g('lead_max'))
        self.hold = float(g('hold'))
        self.tcp = float(g('tcp_offset'))
        self.tol = float(g('tolerance'))

        # Meeting point, and the direction the handle moves there (tangent)
        self.meet = np.array([self.cx + self.r * math.cos(self.theta_m),
                              self.cy + self.r * math.sin(self.theta_m),
                              self.hz])
        self.tangent = np.array([-math.sin(self.theta_m),
                                 math.cos(self.theta_m), 0.0])

        # Gripper orientation: approach (end-effector z) pointing down, finger
        # axis along the tangent, so a timing error slides the bar along the
        # gripper's opening.
        z = np.array([0.0, 0.0, -1.0])
        f = self.tangent
        if g('finger_axis') == 'y':
            x, y = np.cross(f, z), f                   # x = y cross z
        else:
            x, y = f, np.cross(z, f)                   # y = z cross x
        self.R_goal = np.column_stack([x, y, z])

        self.urdf = None
        self.chain = None
        self.q_now = None
        self.joint_hist = []             # (t, joint angles), last 5 s
        self.q_hover = self.q_grasp = None
        self.state = None                # latest [t_meas, theta, omega, t_now]
        self.prev_gap = None
        self.truth = []                  # (t, x, y, z) of the real handle
        self.phase = 'setup'
        self.trial = 0
        self.t_phase = None
        self.t_meet = None
        self.t_commit = None
        self.commit_info = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description',
                                 self.on_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Float64MultiArray, '/target/handle_state',
                                 self.on_state, 10)
        self.create_subscription(PoseStamped, '/target/handle_true',
                                 self.on_truth, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.log = None
        if g('log_file'):
            folder = os.path.dirname(g('log_file'))
            if folder:
                os.makedirs(folder, exist_ok=True)
            self.log_fh = open(g('log_file'), 'w', newline='')
            self.log = csv.writer(self.log_fh)
            self.log.writerow([
                'method', 'spin_period', 'trial', 't_commit', 't_meet',
                'lead', 'omega_est', 'err_along', 'err_across', 'err_vertical',
                'err_dist', 'success'])
        self.spin_period = float(g('spin_period'))
        self.done = False
        self.create_timer(0.01, self.step)

    # ------------------------------------------------------------ inputs

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_urdf(self, msg):
        self.urdf = msg.data

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.q_now = np.array([pos[j] for j in ARM_JOINTS])
            self.t_joints = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            self.joint_hist.append((self.t_joints, self.q_now.copy()))
            while self.joint_hist and self.joint_hist[0][0] < self.t_joints - 5.0:
                self.joint_hist.pop(0)          # keep the last 5 s

    def on_truth(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.position
        if not self.truth or t > self.truth[-1][0]:
            self.truth.append((t, p.x, p.y, p.z))
            self.truth = self.truth[-1000:]

    def on_state(self, msg):
        if len(msg.data) >= 4:
            self.state = list(msg.data[:4])

    # ------------------------------------------------------------ helpers

    def grasp_point(self, q):
        """Where the middle of the gripper fingers is, for joint angles q."""
        T = self.chain.fk(q)
        return T[:3, 3] + self.tcp * T[:3, 2]

    def goal_for(self, p):
        """End-effector pose that puts the grasp point at p."""
        return transform(self.R_goal, p - self.tcp * self.R_goal[:, 2])

    def send(self, points, start=None):
        """points: list of (joint angles, seconds after start)."""
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        if start is not None:
            msg.header.stamp = Time(nanoseconds=int(start * 1e9)).to_msg()
        for q, t in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [0.0] * 6
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(pt)
        self.traj_pub.publish(msg)

    def truth_at(self, t):
        h = self.truth
        for a, b in zip(h, h[1:]):
            if a[0] <= t <= b[0]:
                w = (t - a[0]) / (b[0] - a[0])
                return np.array([a[i] + w * (b[i] - a[i]) for i in (1, 2, 3)])
        return None

    def joints_at(self, t):
        h = self.joint_hist
        for a, b in zip(h, h[1:]):
            if a[0] <= t <= b[0]:
                w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return a[1] + w * (b[1] - a[1])
        return None

    # ------------------------------------------------------------ main loop

    def step(self):
        now = self.now()
        if self.phase == 'setup':
            self.setup(now)
        elif self.phase == 'to_hover':
            if now - self.t_phase > 4.0:
                self.phase = 'wait'
                self.prev_gap = None
                self.get_logger().info(
                    f'Trial {self.trial + 1}/{self.trials}: waiting for the handle')
        elif self.phase == 'wait':
            self.try_commit(now)
        elif self.phase == 'descending':
            if now > self.t_meet + self.hold + 0.3:
                self.measure()
        elif self.phase == 'rising':
            if now - self.t_phase > 1.5:
                self.trial += 1
                if self.trial >= self.trials:
                    self.get_logger().info('All trials done.')
                    self.done = True
                    self.phase = 'finished'
                else:
                    self.phase = 'wait'
                    self.prev_gap = None
                    self.get_logger().info(
                        f'Trial {self.trial + 1}/{self.trials}: waiting for the handle')

    def setup(self, now):
        if self.urdf is None or self.q_now is None or now <= 0:
            return
        self.chain = Chain(self.urdf, EE_LINK)
        hover = self.meet + np.array([0, 0, float(
            self.get_parameter('hover_height').value)])
        seeds = [self.q_now, np.zeros(6),
                 np.array([0, 0.35, 2.5, 0, -1.3, 0]),     # arm folded forward
                 np.array([0, -0.35, -2.5, 0, 1.3, 0])]
        rng = np.random.default_rng(0)
        for _ in range(20):
            seeds.append(rng.uniform(-2.0, 2.0, 6))
        self.q_hover = solve_ik(self.chain, self.goal_for(hover), seeds)
        if self.q_hover is None:
            self.get_logger().error('No IK solution for the hover pose.')
            self.done = True
            return
        self.q_grasp = solve_ik(self.chain, self.goal_for(self.meet),
                                [self.q_hover] + seeds)
        if self.q_grasp is None:
            self.get_logger().error('No IK solution for the grasp pose.')
            self.done = True
            return
        err_h = np.linalg.norm(self.grasp_point(self.q_hover) - hover)
        err_g = np.linalg.norm(self.grasp_point(self.q_grasp) - self.meet)
        self.get_logger().info(
            f'Meeting point {np.round(self.meet, 3)}; IK check: hover '
            f'{err_h * 1000:.2f} mm, grasp {err_g * 1000:.2f} mm off. '
            f'Hover joints {np.round(self.q_hover, 3)}; grasp joints '
            f'{np.round(self.q_grasp, 3)}')
        self.send([(self.q_hover, 3.0)])
        self.phase, self.t_phase = 'to_hover', now

    def try_commit(self, now):
        """Decide whether to start the descent now (scheduled start time)."""
        if self.state is None:
            return
        t_k, theta, omega, _ = self.state
        gap = (self.theta_m - theta) % TWO_PI     # angle still to travel
        if self.method == 'reactive':
            # No prediction: go when the measured handle reaches the point,
            # i.e. when the remaining gap wraps from ~0 to ~2*pi.
            crossed = self.prev_gap is not None and gap > self.prev_gap + math.pi
            self.prev_gap = gap
            if crossed:
                self.commit(now, now + self.descent, omega, self.descent,
                            scheduled=False)
            return
        if omega <= 1e-3:
            return
        t_meet = t_k + gap / omega
        lead = t_meet - now
        if self.descent + self.lead_min <= lead <= self.descent + self.lead_max:
            self.commit(now, t_meet, omega, lead)

    def commit(self, now, t_meet, omega, lead, scheduled=True):
        # Scheduled: the controller waits and starts the descent exactly at
        # t_meet - descent. Reactive: start straight away.
        start = t_meet - self.descent if scheduled else None
        self.send([(self.q_hover, 0.0), (self.q_grasp, self.descent)], start=start)
        self.t_commit, self.t_meet = now, t_meet
        self.commit_info = (omega, lead)
        self.phase = 'descending'

    def measure(self):
        q = self.joints_at(self.t_meet)
        h = self.truth_at(self.t_meet)
        if q is None or h is None:
            self.get_logger().warn('No data at the meeting time; trial skipped.')
        else:
            e = self.grasp_point(q) - h
            hx, hy = h[0] - self.cx, h[1] - self.cy
            ang = math.atan2(hy, hx)
            tang = np.array([-math.sin(ang), math.cos(ang), 0.0])
            radial = np.array([math.cos(ang), math.sin(ang), 0.0])
            along, across, vert = float(e @ tang), float(e @ radial), float(e[2])
            dist = float(np.linalg.norm(e))
            ok = dist <= self.tol
            omega, lead = self.commit_info
            self.get_logger().info(
                f'Trial {self.trial + 1}: error {dist * 1000:.1f} mm '
                f'(along {along * 1000:+.1f}, across {across * 1000:+.1f}, '
                f'vertical {vert * 1000:+.1f}) -> '
                f'{"CAPTURED" if ok else "missed"}')
            if self.log:
                self.log.writerow([
                    self.method, self.spin_period, self.trial + 1,
                    f'{self.t_commit:.3f}', f'{self.t_meet:.3f}', f'{lead:.3f}',
                    f'{omega:.4f}', f'{along:.5f}', f'{across:.5f}',
                    f'{vert:.5f}', f'{dist:.5f}', int(ok)])
                self.log_fh.flush()
        self.send([(self.q_hover, 1.0)])
        self.phase, self.t_phase = 'rising', self.now()

    def close(self):
        if self.log:
            self.log_fh.close()


def main(args=None):
    rclpy.init(args=args)
    node = Capture()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
