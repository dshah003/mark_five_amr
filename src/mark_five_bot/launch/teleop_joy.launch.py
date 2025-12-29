"""
Joystick Teleoperation Launch File

Launches joystick driver and teleop nodes for manual control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('mark_five_bot')

    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'teleop.yaml')

    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # Joy node (joystick driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[config_file],
    )

    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
