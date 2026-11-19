from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'mode',
            default_value='orbit'
        ),

        DeclareLaunchArgument(
            'radius',
            default_value='0.10'
        ),

        DeclareLaunchArgument(
            'orbit_period',
            default_value='5.0'
        ),

        DeclareLaunchArgument(
            'spin_period',
            default_value='3.0'
        ),

        # Target centre position (m), relative to the arm base.
        # 0.5 m in front keeps the handle well inside the Gen3's reach;
        # 0.20 m up leaves room for the gripper to approach from above.
        DeclareLaunchArgument(
            'x',
            default_value='0.5'
        ),

        DeclareLaunchArgument(
            'y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.20'
        ),

        Node(
            package='rotating_target',
            executable='target_motion',
            name='target_motion',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mode': LaunchConfiguration('mode'),
                'radius': ParameterValue(
                    LaunchConfiguration('radius'),
                    value_type=float
                ),
                'orbit_period': ParameterValue(
                    LaunchConfiguration('orbit_period'),
                    value_type=float
                ),
                'spin_period': ParameterValue(
                    LaunchConfiguration('spin_period'),
                    value_type=float
                ),
                'x': ParameterValue(
                    LaunchConfiguration('x'),
                    value_type=float
                ),
                'y': ParameterValue(
                    LaunchConfiguration('y'),
                    value_type=float
                ),
                'z': ParameterValue(
                    LaunchConfiguration('z'),
                    value_type=float
                )
            }]
        )

    ])
