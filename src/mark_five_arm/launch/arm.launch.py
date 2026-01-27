#!/usr/bin/env python3
"""
Launch file for Mark Five robotic arm.

This launch file starts the arm controller node with PCA9685 PWM driver.

Usage:
    ros2 launch mark_five_arm arm.launch.py
    ros2 launch mark_five_arm arm.launch.py config:=/path/to/custom_config.yaml

Arguments:
    config (str): Path to arm configuration YAML file
                  Default: Uses config/arm_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            FindPackageShare('mark_five_arm'),
            'config',
            'arm_params.yaml'
        ]),
        description='Path to arm configuration YAML file'
    )

    # Get launch configuration
    config_file = LaunchConfiguration('config')

    # Arm controller node
    arm_controller_node = Node(
        package='mark_five_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        arm_controller_node,
    ])
