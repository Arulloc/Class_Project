
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState


class TargetMotion(Node):

    def __init__(self):
        super().__init__('target_motion')

        # Configurable ROS parameters
        self.declare_parameter('mode', 'orbit')
        self.declare_parameter('radius', 0.10)
        self.declare_parameter('orbit_period', 5.0)
        self.declare_parameter('spin_period', 3.0)
        self.declare_parameter('x', 0.75)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.05)

        self.mode = self.get_parameter('mode').value
        self.radius = float(self.get_parameter('radius').value)
        self.orbit_period = float(
            self.get_parameter('orbit_period').value)
        self.spin_period = float(
            self.get_parameter('spin_period').value)

        self.x0 = float(self.get_parameter('x').value)
        self.y0 = float(self.get_parameter('y').value)
        self.z0 = float(self.get_parameter('z').value)

        if self.mode not in ('spin', 'orbit', 'both'):
            raise ValueError('Mode must be spin, orbit, or both')

        if self.orbit_period <= 0 or self.spin_period <= 0:
            raise ValueError('Rotation periods must be positive')

        if self.radius < 0:
            raise ValueError('Radius cannot be negative')

        self.model_name = 'rotating_target'

        self.model_path = (
            Path.home()
            / 'class_project/models/rotating_target/model.sdf'
        )

        self.spawn_client = self.create_client(
            SpawnEntity, '/spawn_entity')

        self.state_client = self.create_client(
            SetEntityState, '/set_entity_state')

        self.spawned = False
        self.start_time = None
        self.spawn_future = None
        self.state_future = None
        self.spawn_requested = False

        # Run at 20 Hz. Gazebo simulation time is enabled
        # using the use_sim_time ROS parameter.
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info(
            f'Target controller initialized: {self.mode}')

    def spawn_target(self):
        if self.spawn_requested:
            return

        if not self.spawn_client.service_is_ready():
            return

        if not self.state_client.service_is_ready():
            return

        if not self.model_path.exists():
            self.get_logger().error(
                f'Model file not found: {self.model_path}')
            self.timer.cancel()
            return

        request = SpawnEntity.Request()
        request.name = self.model_name
        request.xml = self.model_path.read_text()
        request.robot_namespace = ''

        request.initial_pose.position.x = self.x0
        request.initial_pose.position.y = self.y0
        request.initial_pose.position.z = self.z0
        request.initial_pose.orientation.w = 1.0
        request.reference_frame = 'world'

        self.spawn_requested = True
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.on_spawn)

        self.get_logger().info('Spawning target in Gazebo...')

    def on_spawn(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spawn failed: {exc}')
            self.timer.cancel()
            return

        if not result.success:
            self.get_logger().error(
                f'Gazebo rejected spawn: {result.status_message}')
            self.timer.cancel()
            return

        self.spawned = True
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'Target spawned. Starting mode: {self.mode}')

    def update(self):
        if not self.spawned:
            self.spawn_target()
            return

        if not self.state_client.service_is_ready():
            return

        # Avoid accumulating service requests if Gazebo is slow.
        if self.state_future is not None:
            if not self.state_future.done():
                return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        # Handle simulation clock resets.
        if t < 0:
            self.start_time = now
            t = 0.0

        x = self.x0
        y = self.y0
        z = self.z0
        yaw = 0.0

        # Mode 2 and 3: circular orbit around a fixed centre.
        # The orbit starts at x0, y0.
        if self.mode in ('orbit', 'both'):
            angle = 2.0 * math.pi * t / self.orbit_period

            x = self.x0 + self.radius * (math.cos(angle) - 1.0)
            y = self.y0 + self.radius * math.sin(angle)

        # Mode 1 and 3: spin around the cube's own Z-axis.
        if self.mode in ('spin', 'both'):
            yaw = 2.0 * math.pi * t / self.spin_period

        request = SetEntityState.Request()

        state = request.state
        state.name = self.model_name
        state.reference_frame = 'world'

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        # Convert yaw angle into a quaternion.
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        # The static target follows a prescribed trajectory.
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        self.state_future = self.state_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    node = TargetMotion()

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
