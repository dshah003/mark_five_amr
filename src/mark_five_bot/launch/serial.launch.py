"""
Serial Communication Launch File

Launches the serial bridge node for Arduino communication.
The Arduino must be running the simple serial protocol firmware.

serial_cmd_vel_topic: the velocity topic serial_bridge drives the motors from.
Default /cmd_vel_safe (collision_monitor output, no watchdog). bringup.launch.py
and robot.launch.py set /cmd_vel_gated when they launch the mark_five_safety
watchdog (use_safety:=true). The arg name is deliberately specific: launch
configurations are global across includes (see the lidar `port` collision).
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

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'serial_cmd_vel_topic',
        default_value='/cmd_vel_safe',
        description='Velocity topic serial_bridge subscribes to (/cmd_vel_gated behind the safety watchdog)'
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
            'cmd_vel_topic': LaunchConfiguration('serial_cmd_vel_topic'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        cmd_vel_topic_arg,
        serial_bridge_node,
    ])
