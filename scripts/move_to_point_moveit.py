#!/usr/bin/env python3
# move_to_point_moveit.py: ask MoveIt to move the Kinova Gen3's end effector to an (x, y, z) point.
# Usage: python3 move_to_point_moveit.py X Y Z [--rpy R P Y] [--plan-only] [--speed 0-1]

# Standard library: argument parsing, trig for the quaternion, and exiting with an error
import argparse
import math
import sys

# ROS 2 Python client: node, action client, and the MoveIt/geometry message types we fill in
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, PositionConstraint,
                             OrientationConstraint, MoveItErrorCodes)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion

# Names from Kinova's MoveIt config: planning group, link that must reach the point, reference frame, action
GROUP = "manipulator"
EE_LINK = "end_effector_link"
FRAME = "base_link"
ACTION = "/move_action"


# Convert roll/pitch/yaw angles (rad) into the quaternion format ROS uses for orientation
def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy,
                      y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy,
                      w=cr * cp * cy + sr * sp * sy)


# Pack the target point, optional orientation, and planning settings into one MoveGroup goal
def build_goal(xyz, rpy, plan_only, speed):
    goal = MoveGroup.Goal()
    req = goal.request

    # Planning settings: which group, start from current pose, attempts, time limit, speed scaling
    req.group_name = GROUP
    req.start_state.is_diff = True
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = speed
    req.max_acceleration_scaling_factor = speed

    # Container for every condition the final pose must satisfy
    c = Constraints()

    # Position goal: end effector must finish inside a 5 mm sphere centred on the target point
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE_LINK
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
    centre = Pose()
    centre.position.x, centre.position.y, centre.position.z = xyz
    centre.orientation.w = 1.0
    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(centre)
    pc.weight = 1.0
    c.position_constraints.append(pc)

    # Orientation goal (only with --rpy): gripper must face this way within about ±3° per axis
    if rpy is not None:
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = EE_LINK
        oc.orientation = rpy_to_quat(*rpy)
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

    # Attach the constraints and choose plan-only vs plan-and-execute, with up to 2 replans
    req.goal_constraints.append(c)
    goal.planning_options.plan_only = plan_only
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 2
    return goal


def main():
    # Read the target point and options from the command line
    p = argparse.ArgumentParser()
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("z", type=float)
    p.add_argument("--rpy", nargs=3, type=float, help="roll pitch yaw (rad)")
    p.add_argument("--plan-only", action="store_true")
    p.add_argument("--speed", type=float, default=0.3, help="0-1 velocity scaling")
    a = p.parse_args()

    # Start ROS, create this node, and connect to MoveIt's move_group action server
    rclpy.init()
    node = Node("move_to_point_moveit")
    client = ActionClient(node, MoveGroup, ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        sys.exit(f"No {ACTION} server. Is MoveIt (move_group) running?")

    # Build the goal and send it, then wait until move_group accepts or rejects it
    goal = build_goal((a.x, a.y, a.z), a.rpy, a.plan_only, a.speed)
    print(f"Asking MoveIt to move {EE_LINK} to {a.x, a.y, a.z} in {FRAME} ...")
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        sys.exit("Goal rejected by move_group.")

    # Wait for MoveIt to finish planning (and moving), then read its result code
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    code = res_fut.result().result.error_code.val

    # Report success, or turn the numeric error code into its name (e.g. NO_IK_SOLUTION)
    if code == MoveItErrorCodes.SUCCESS:
        print("Planned only (see RViz)." if a.plan_only else "Reached target.")
    else:
        names = {v: k for k, v in vars(MoveItErrorCodes).items()
                 if k.isupper() and isinstance(v, int)}
        print(f"MoveIt failed: {names.get(code, code)}")
        if code == MoveItErrorCodes.NO_IK_SOLUTION:
            print("Point is probably out of reach, or the orientation is impossible there.")

    # Clean up the node and shut ROS down
    node.destroy_node()
    rclpy.shutdown()


# Run main() only when the file is executed directly, not when imported
if __name__ == "__main__":
    main()
