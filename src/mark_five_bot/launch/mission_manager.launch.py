"""
Mission Manager Launch File for Mark Five AMR

Launches the waypoint mission manager node that provides service-based
API for controlling multi-waypoint autonomous missions.

Usage:
  ros2 launch mark_five_bot mission_manager.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mark_five_bot')
    config_file = os.path.join(pkg_share, 'config', 'mission_manager.yaml')

    mission_manager_node = Node(
        package='mark_five_bot',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        mission_manager_node,
    ])
