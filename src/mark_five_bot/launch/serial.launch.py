"""
Serial Communication Launch File

Launches the serial bridge node for Arduino communication.
The Arduino must be running the simple serial protocol firmware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    # Serial bridge node
    serial_bridge_node = Node(
        package='mark_five_bot',
        executable='serial_bridge.py',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        serial_bridge_node,
    ])
