#!/usr/bin/env python3
"""demo_pickup.py: for the class demo - make the captured target go with the gripper.

The simulated gripper cannot close, so a real grasp is not
possible in Gazebo Classic. For the demo, this program imitates one:

  1. It watches the capture program's descent command to learn the meeting
     time, and at that moment checks (as in the experiments) whether the
     grasp point is within the 36 mm tolerance of the true handle position.
  2. If it is, it stops the target's spinning motion and from then on moves
     the target together with the gripper, keeping the target exactly where it
     was relative to the gripper at the moment of capture. The arm's rise and
     the lift afterwards then carry the target up.
  3. If it is not, nothing is attached and the target keeps spinning.

It writes the word CAPTURED or MISSED to the status file so run_demo.sh knows
what happened. The target is moved by setting its pose in Gazebo, the same
way the target's own motion program moves it.

Run (started by scripts/run_demo.sh, not by hand):
  python3 scripts/demo_pickup.py --ros-args -p use_sim_time:=true \
      -p status_file:=results/demo/status.txt
"""

import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from gazebo_msgs.srv import GetEntityState, SetEntityState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture import ARM_JOINTS, EE_LINK, Chain, rot_to_quat   # same arm model code


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class DemoPickup(Node):

    def __init__(self):
        super().__init__('demo_pickup')
        P = self.declare_parameter
        P('target_name', 'rotating_target')   # the target's name in Gazebo
        P('center_x', 0.5)                     # spin axis (m), as the predictor
        P('center_y', 0.0)
        P('tcp_offset', 0.12)                  # grasp point beyond end_effector_link (m)
        P('tolerance', 0.036)                  # capture tolerance (m), report 6.2
        P('status_file', '')
        g = lambda n: self.get_parameter(n).value
        self.name = g('target_name')
        self.c = np.array([float(g('center_x')), float(g('center_y'))])
        self.tcp = float(g('tcp_offset'))
        self.tol = float(g('tolerance'))
        self.status_file = g('status_file')

        self.chain = None
        self.truth = []                 # (t, x, y, z) of the handle
        self.joint_hist = []            # (t, q)
        self.q_now = None
        self.t_meet = None
        self.cube0 = None               # target pose read from Gazebo
        self.target_reading = None      # (time, Gazebo's answer) waiting to be compared
        self.t_read = 0.0
        self.read_busy = False
        self.t_ask = 0.0
        self.yaw_offset = None          # target yaw minus handle angle
        self.T_grip_cube = None         # target pose in the gripper frame once attached
        self.busy = False
        self.last_set = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(PoseStamped, '/target/handle_true', self.on_truth, 50)
        self.create_subscription(JointTrajectory,
                                 '/joint_trajectory_controller/joint_trajectory',
                                 self.on_traj, 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_timer(1.0, self.read_target_once)
        self.write_status('WAITING')
        self.get_logger().info('Demo pick-up ready: waiting for a capture attempt.')

    # ------------------------------------------------------------ inputs

    def write_status(self, word):
        if self.status_file:
            with open(self.status_file, 'w') as f:
                f.write(word + '\n')

    def on_urdf(self, msg):
        if self.chain is None:
            self.chain = Chain(msg.data, EE_LINK)

    def on_truth(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.position
        if not self.truth or t > self.truth[-1][0]:
            self.truth.append((t, p.x, p.y, p.z))
            self.truth = self.truth[-300:]
            self.use_target_reading()

    def on_traj(self, msg):
        # The capture program's descent is the only command with three points
        # (hover, grasp, hover); the grasp point is reached at the meeting time.
        if len(msg.points) == 3 and self.T_grip_cube is None:
            start = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            d = msg.points[1].time_from_start
            self.t_meet = start + d.sec + d.nanosec * 1e-9
            self.get_logger().info(f'Capture attempt: meeting time {self.t_meet:.2f} s')

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in ARM_JOINTS):
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.q_now = np.array([pos[j] for j in ARM_JOINTS])
        self.joint_hist.append((t, self.q_now.copy()))
        while self.joint_hist and self.joint_hist[0][0] < t - 5.0:
            self.joint_hist.pop(0)
        if self.t_meet is not None and self.T_grip_cube is None:
            self.check_capture()
        elif self.T_grip_cube is not None:
            self.carry()

    # ------------------------------------------------------------ helpers

    def grip_pose(self, q):
        T = self.chain.fk(q)
        G = T.copy()
        G[:3, 3] = T[:3, 3] + self.tcp * T[:3, 2]     # grasp point between the fingers
        return G

    def at_time(self, hist, t):
        for a, b in zip(hist, hist[1:]):
            if a[0] <= t <= b[0]:
                w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return a, b, w
        return None

    def handle_at(self, t):
        r = self.at_time(self.truth, t)
        if r is None:
            return None
        a, b, w = r
        return np.array([a[i] + w * (b[i] - a[i]) for i in (1, 2, 3)])

    def joints_at(self, t):
        r = self.at_time(self.joint_hist, t)
        if r is None:
            return None
        a, b, w = r
        return a[1] + w * (b[1] - a[1])

    def read_target_once(self):
        """Read the target's pose from Gazebo once, to learn how its yaw
        relates to the handle angle and where its centre is."""
        if self.cube0 is not None:
            return
        if self.target_reading is not None:
            if time.monotonic() - self.t_read < 3.0:
                return                  # still waiting to compare the last reading
            self.target_reading = None
        if not self.get_cli.service_is_ready():
            self.get_logger().info('Waiting for Gazebo to offer its state service...')
            return
        if self.read_busy:
            if time.monotonic() - self.t_ask < 2.0:
                return
            # ROS 2 sometimes loses a request sent just after a service is
            # found; if there is no answer within 2 s, ask again.
            self.get_logger().info('No answer from Gazebo; asking again.')
        self.t_ask = self.t_read = time.monotonic()
        req = GetEntityState.Request()
        req.name, req.reference_frame = self.name, 'world'
        self.read_busy = True
        self.get_cli.call_async(req).add_done_callback(self.on_target_read)

    def on_target_read(self, fut):
        self.read_busy = False
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn(f'Could not read {self.name} from Gazebo yet.')
            return
        # The true handle position arrives only about 10 times a second, so
        # the comparison waits until a reading from after this moment arrives.
        self.target_reading = (Time.from_msg(res.header.stamp).nanoseconds * 1e-9, res)
        self.use_target_reading()

    def use_target_reading(self):
        if self.cube0 is not None or self.target_reading is None:
            return
        t, res = self.target_reading
        if t <= 0 and self.truth:
            t = self.truth[-1][0]       # no time stamp given: use the latest reading
        if not self.truth or self.truth[-1][0] < t:
            return                      # wait for the next handle reading
        h = self.handle_at(t)
        if h is None:
            self.target_reading = None  # too old to compare: read Gazebo again
            return
        p, o = res.state.pose.position, res.state.pose.orientation
        R = quat_to_rot(o.x, o.y, o.z, o.w)
        yaw = math.atan2(R[1, 0], R[0, 0])
        ang = math.atan2(h[1] - self.c[1], h[0] - self.c[0])
        self.yaw_offset = yaw - ang
        self.cube0 = dict(pos=np.array([p.x, p.y, p.z]), R=R, yaw=yaw)
        self.get_logger().info('Target pose read from Gazebo.')

    # ------------------------------------------------------------ capture and carry

    def check_capture(self):
        if self.chain is None or self.cube0 is None:
            if not getattr(self, 'warned', False):
                self.warned = True
                self.get_logger().warn('Meeting time reached but the target pose has '
                                       'not been read from Gazebo yet; cannot attach.')
            return
        # Wait until both the joints and the true handle position are known
        # on both sides of the meeting time
        if not self.truth or self.truth[-1][0] < self.t_meet:
            return
        q = self.joints_at(self.t_meet)
        h = self.handle_at(self.t_meet)
        if q is None or h is None:
            return
        G = self.grip_pose(q)
        err = float(np.linalg.norm(G[:3, 3] - h))
        if err > self.tol:
            self.get_logger().info(f'MISSED: grasp point {err * 1000:.1f} mm from the handle.')
            self.write_status('MISSED')
            self.t_meet = None
            return
        # Captured: stop the spinning and fix the target to the gripper
        subprocess.run(['pkill', '-f', 'target_motion'], check=False)
        ang = math.atan2(h[1] - self.c[1], h[0] - self.c[0])
        R_cube = rot_z(ang + self.yaw_offset - self.cube0['yaw']) @ self.cube0['R']
        T_cube = np.eye(4)
        T_cube[:3, :3], T_cube[:3, 3] = R_cube, self.cube0['pos']
        self.T_grip_cube = np.linalg.inv(G) @ T_cube
        self.get_logger().info(
            f'CAPTURED: grasp point {err * 1000:.1f} mm from the handle. '
            'Target now moves with the gripper.')
        self.write_status('CAPTURED')

    def carry(self):
        # About 30 updates a second, one request at a time
        if time.monotonic() - self.last_set < 0.033:
            return
        if self.busy and time.monotonic() - self.last_set < 0.5:
            return                      # previous move not answered yet (resend after 0.5 s)
        if not self.set_cli.service_is_ready():
            return
        T = self.grip_pose(self.q_now) @ self.T_grip_cube
        x, y, z, w = rot_to_quat(T[:3, :3])
        req = SetEntityState.Request()
        req.state.name = self.name
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = \
            map(float, T[:3, 3])
        req.state.pose.orientation.x, req.state.pose.orientation.y = float(x), float(y)
        req.state.pose.orientation.z, req.state.pose.orientation.w = float(z), float(w)
        self.busy = True
        self.last_set = time.monotonic()
        self.set_cli.call_async(req).add_done_callback(self.on_set)

    def on_set(self, fut):
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = DemoPickup()
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
