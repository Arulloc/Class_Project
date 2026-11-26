#!/usr/bin/env python3
"""tag_detector.py: Stage C - find the handle with the external camera.

For every camera image it:
  1. finds AprilTag 36h10 ID 0 (the tag on top of the target),
  2. works out the tag's position and rotation relative to the camera from
     its four corners and its known size (8 cm),
  3. converts that into the world frame using the camera's known pose,
  4. places the handle 0.13 m from the tag centre in the direction the tag
     is turned, at the height of the grip bar, and
  5. publishes it on /target/handle_measured, time-stamped with the time the
     image was taken. The predictor reads this instead of the ground truth.

It also logs every detection next to the ground truth at the same instant,
so the camera's own error and delay can be reported (6.1, 7.4).

Tag orientation on the cube: Gazebo may paint the tag onto the cube top
rotated by a multiple of 90 degrees. At start-up the detector compares its
first 30 detections with the ground truth once, picks the nearest multiple
of 90 degrees, logs it, and uses it from then on (set yaw_offset to a value
in degrees to skip this).

Uses OpenCV's built-in AprilTag 36h10 dictionary (no extra install).

Run:
  python3 scripts/tag_detector.py --ros-args -p use_sim_time:=true \
      -p cam_x:=0.75 -p cam_y:=0.0 -p cam_z:=0.75 -p cam_pitch:=1.107 \
      -p cam_yaw:=3.1416 -p log_file:=results/stageC/vision_test.csv
"""

import csv
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

# Camera link (x forward, y left, z up) -> optical frame (z forward, x right,
# y down): columns are the optical axes written in the link frame.
R_LINK_OPTICAL = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def make_detector():
    """AprilTag 36h10 detector that works with old and new OpenCV versions."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h10)
    if hasattr(aruco, 'ArucoDetector'):                 # OpenCV 4.7 and newer
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        det = aruco.ArucoDetector(dictionary, params)
        return lambda img: det.detectMarkers(img)[:2]
    params = aruco.DetectorParameters_create()          # OpenCV 4.5 (Ubuntu 22.04)
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return lambda img: aruco.detectMarkers(img, dictionary, parameters=params)[:2]


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')
        P = self.declare_parameter
        P('image_topic', '/ext_camera/image_raw')
        P('info_topic', '/ext_camera/camera_info')
        P('cam_x', 0.75)                 # camera pose in the world (m, rad),
        P('cam_y', 0.0)                  # the same numbers it was spawned with
        P('cam_z', 0.75)
        P('cam_roll', 0.0)
        P('cam_pitch', 1.107)
        P('cam_yaw', math.pi)
        P('tag_id', 0)
        P('tag_size', 0.08)              # black square edge (m)
        P('tag_height', 0.0513)          # tag surface above the cube centre (m)
        P('handle_offset', 0.13)         # grip bar from the cube centre (m)
        P('yaw_offset', -999.0)          # degrees; -999 = find it at start-up
        P('log_file', '')
        g = lambda n: self.get_parameter(n).value
        self.cam_pos = np.array([g('cam_x'), g('cam_y'), g('cam_z')], float)
        self.R_wo = rot_rpy(g('cam_roll'), g('cam_pitch'), g('cam_yaw')) @ R_LINK_OPTICAL
        self.tag_id = int(g('tag_id'))
        s = float(g('tag_size')) / 2
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                            dtype=np.float64)
        self.tag_h = float(g('tag_height'))
        self.r = float(g('handle_offset'))
        yo = float(g('yaw_offset'))
        self.yaw_offset = None if yo < -900 else math.radians(yo)
        self.calib = []
        self.detect = make_detector()
        self.K = None
        self.truth = []
        self.n_images = self.n_found = 0
        self.errs, self.lags = [], []

        self.pub = self.create_publisher(PoseStamped, '/target/handle_measured', 10)
        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, 10)
        self.create_subscription(Image, g('image_topic'), self.on_image, 10)
        self.create_subscription(PoseStamped, '/target/handle_true', self.on_truth, 10)
        self.create_timer(5.0, self.report)

        self.log = None
        if g('log_file'):
            folder = os.path.dirname(g('log_file'))
            if folder:
                os.makedirs(folder, exist_ok=True)
            self.log_fh = open(g('log_file'), 'w', newline='')
            self.log = csv.writer(self.log_fh)
            self.log.writerow(['t_image', 't_published', 'latency', 'meas_x',
                               'meas_y', 'meas_z', 'true_x', 'true_y', 'true_z',
                               'error_m', 'angle_error_deg'])

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def on_truth(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.position
        if not self.truth or t > self.truth[-1][0]:
            self.truth.append((t, p.x, p.y, p.z))
            self.truth = self.truth[-300:]

    def truth_at(self, t):
        for a, b in zip(self.truth, self.truth[1:]):
            if a[0] <= t <= b[0]:
                w = (t - a[0]) / (b[0] - a[0])
                return np.array([a[i] + w * (b[i] - a[i]) for i in (1, 2, 3)])
        return None

    def on_image(self, msg):
        if self.K is None:
            return
        self.n_images += 1
        t_img = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if ch == 3 else img[:, :, 0]
        corners, ids = self.detect(gray)
        if ids is None:
            return
        for c, i in zip(corners, ids.flatten()):
            if int(i) == self.tag_id:
                self.use(c.reshape(4, 2).astype(np.float64), t_img)
                return

    def use(self, corners, t_img):
        ok, rvec, tvec = cv2.solvePnP(self.obj, corners, self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        R_ct, _ = cv2.Rodrigues(rvec)
        p_tag = self.R_wo @ tvec.flatten() + self.cam_pos
        x_axis = self.R_wo @ R_ct[:, 0]
        yaw = math.atan2(x_axis[1], x_axis[0])
        truth = self.truth_at(t_img)

        if self.yaw_offset is None:
            # One-off: which way round is the tag painted on the cube?
            if truth is not None:
                true_ang = math.atan2(truth[1] - p_tag[1], truth[0] - p_tag[0])
                self.calib.append(wrap(true_ang - yaw))
            if len(self.calib) >= 30:
                mean = math.atan2(sum(math.sin(a) for a in self.calib),
                                  sum(math.cos(a) for a in self.calib))
                self.yaw_offset = math.radians(90 * round(math.degrees(mean) / 90))
                self.get_logger().info(
                    f'Tag orientation on the cube: handle is '
                    f'{math.degrees(self.yaw_offset):.0f} deg from the tag x-axis '
                    f'(measured {math.degrees(mean):.1f} deg, rounded to 90)')
            return

        ang = yaw + self.yaw_offset
        h = np.array([p_tag[0] + self.r * math.cos(ang),
                      p_tag[1] + self.r * math.sin(ang),
                      p_tag[2] - self.tag_h])
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = Time(nanoseconds=int(t_img * 1e9)).to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, h)
        msg.pose.orientation.z = math.sin(ang / 2)
        msg.pose.orientation.w = math.cos(ang / 2)
        self.pub.publish(msg)
        self.n_found += 1

        t_pub = self.get_clock().now().nanoseconds * 1e-9
        lag = t_pub - t_img
        self.lags.append(lag)
        if truth is not None:
            err = float(np.linalg.norm(h - truth))
            cx, cy = p_tag[0], p_tag[1]
            a_err = math.degrees(wrap(math.atan2(h[1] - cy, h[0] - cx)
                                      - math.atan2(truth[1] - cy, truth[0] - cx)))
            self.errs.append(err)
            if self.log:
                self.log.writerow([f'{t_img:.3f}', f'{t_pub:.3f}', f'{lag:.4f}',
                                   *[f'{v:.5f}' for v in h],
                                   *[f'{v:.5f}' for v in truth],
                                   f'{err:.5f}', f'{a_err:.3f}'])
                self.log_fh.flush()

    def report(self):
        if self.K is None:
            self.get_logger().warn('No camera_info yet. Is the camera spawned?')
            return
        if self.n_images == 0:
            self.get_logger().warn('No images yet.')
            return
        msg = (f'images {self.n_images}, handle measured in {self.n_found} '
               f'({100 * self.n_found / self.n_images:.0f}%)')
        if self.errs:
            msg += (f'; error vs ground truth mean {1000 * np.mean(self.errs):.1f} mm, '
                    f'largest {1000 * max(self.errs):.1f} mm')
        if self.lags:
            msg += f'; processing delay {1000 * np.mean(self.lags):.0f} ms'
        if self.yaw_offset is None:
            msg += f'; finding tag orientation ({len(self.calib)}/30)'
        self.get_logger().info(msg)
        self.errs, self.lags = self.errs[-200:], self.lags[-200:]

    def close(self):
        if self.log:
            self.log_fh.close()


def main(args=None):
    rclpy.init(args=args)
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
