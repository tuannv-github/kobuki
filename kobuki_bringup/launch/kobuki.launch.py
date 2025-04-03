import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    kobuki_node_launch    = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('kobuki_node'), 'launch', 'kobuki_node_mux.launch.py'))
    )

    joy_node =    Node(
            package='joy',
            executable='joy_node',
            name='joy',  # Remapped node name
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.05},
                {'autorepeat_rate': 20.0},
            ],
            arguments=['--log-level', 'info'],
        )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[
                {'axis_linear.x': 1},  # Left stick vertical for linear.x
                {'axis_angular.yaw': 0},  # Left stick horizontal for angular.z
                {'scale_linear.x': 0.5},  # Max linear speed
                {'scale_angular.yaw': 1.5},  # Max angular speed
                {'enable_button': 0},  # Button to enable (e.g., L1)
                {'enable_turbo_button': -1},  # Disable turbo
            ],
            remappings=[('/cmd_vel', '/joy_local_cmd_vel')],  # Remap /cmd_vel to /joy_cmd_vel
        )

    pkg_share = get_package_share_directory('kobuki_bringup')
    config_file = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_file],  # Pass the configuration file
        remappings=[('/cmd_vel_out', '/mux/output/cmd_vel')]  # Remap output topic if needed
    )

    return LaunchDescription([
            kobuki_node_launch,
            joy_node,
            teleop_twist_joy_node,
            twist_mux_node,

        ])
