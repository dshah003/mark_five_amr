#!/usr/bin/env python3
"""
Launch file to visualize the Lite Arm i2 URDF in RViz.

Launches:
  - robot_state_publisher (publishes URDF transforms)
  - joint_state_publisher_gui (manual joint control)
  - RViz2 (visualization)

Usage:
    ros2 launch mark_five_arm display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='lite_arm_i2.urdf.xacro',
        description='URDF file name (in urdf/ directory)'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui'
    )

    # Get launch configuration
    urdf_file = LaunchConfiguration('urdf_file')
    use_gui = LaunchConfiguration('use_gui')

    # Path to URDF file
    urdf_path = PathJoinSubstitution([
        FindPackageShare('mark_five_arm'),
        'urdf',
        urdf_file
    ])

    # Process URDF with xacro
    robot_description = Command(['xacro ', urdf_path])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # RViz2 node
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mark_five_arm'),
        'rviz',
        'display.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        urdf_file_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
