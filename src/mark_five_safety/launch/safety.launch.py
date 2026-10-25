"""
Safety watchdog launch file (runs on the Jetson, included by
mark_five_bot bringup.launch.py when use_safety:=true).

  /cmd_vel_safe -> safety_watchdog -> /cmd_vel_gated -> serial_bridge

Standalone:
  ros2 launch mark_five_safety safety.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('mark_five_safety'),
                          'config', 'safety_watchdog.yaml')
    return LaunchDescription([
        Node(
            package='mark_five_safety',
            executable='safety_watchdog',
            name='safety_watchdog',
            output='screen',
            parameters=[params],
            # Deliberately NOT respawned: a respawned watchdog would come back
            # DISARMED and silently clear a latched trip. A dead watchdog
            # fails safe -- /cmd_vel_gated goes silent and the Arduino's 1 s
            # serial timeout stops the motors; restart bringup to recover.
        ),
    ])
