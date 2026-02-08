#!/usr/bin/env python3
"""
Test launch file for Mark Five robotic arm with GUI control.

This launch file starts:
  - robot_state_publisher (publishes URDF transforms)
  - Arm controller node (PCA9685 driver)
  - Joint State Publisher GUI for manual joint control
  - RViz2 for visualization (optional)

Usage:
    ros2 launch mark_five_arm arm_test.launch.py
    ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false

Use the GUI sliders to test each servo joint and see the arm move in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='lite_arm_i2.urdf.xacro',
        description='URDF file name (in urdf/ directory)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Get launch configuration
    config_file = LaunchConfiguration('config')
    urdf_file = LaunchConfiguration('urdf_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Path to URDF file
    urdf_path = PathJoinSubstitution([
        FindPackageShare('mark_five_arm'),
        'urdf',
        urdf_file
    ])

    # Process URDF with xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

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
            'display.rviz'
        ])],
    )

    return LaunchDescription([
        config_arg,
        urdf_file_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        arm_controller_node,
        joint_state_publisher_gui,
        rviz_node,
    ])
