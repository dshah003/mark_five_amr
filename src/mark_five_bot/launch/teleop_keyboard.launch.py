"""
Keyboard Teleoperation Launch File

Launches keyboard teleop node for manual control.

Note: For keyboard input to work, run this in a terminal with stdin access.
When using bringup.launch.py, consider use_teleop:=false and running teleop
separately: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Keyboard teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'speed': 0.5,
            'turn': 0.5,
        }],
    )

    return LaunchDescription([
        teleop_node,
    ])
