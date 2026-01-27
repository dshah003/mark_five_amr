#!/usr/bin/env python3
"""
Test launch file for Mark Five robotic arm with GUI control.

This launch file starts:
  - Arm controller node (PCA9685 driver)
  - Joint State Publisher GUI for manual joint control
  - RViz2 for visualization (optional)

Usage:
    ros2 launch mark_five_arm arm_test.launch.py
    ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false

Use the GUI sliders to test each servo joint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Get launch configuration
    config_file = LaunchConfiguration('config')
    use_rviz = LaunchConfiguration('use_rviz')

    # Arm controller node
    arm_controller_node = Node(
        package='mark_five_arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )

    # Joint State Publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[
            ('/joint_states', '/arm/joint_commands')
        ],
    )

    # RViz2 (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('mark_five_arm'),
            'rviz',
            'arm.rviz'
        ])],
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        arm_controller_node,
        joint_state_publisher_gui,
        rviz_node,
    ])
