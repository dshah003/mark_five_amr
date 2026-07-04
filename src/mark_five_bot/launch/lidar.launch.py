"""
LDROBOT LD19 Lidar Launch File

Launches the ldlidar_stl_ros2 driver publishing /scan in frame 'base_laser'.

The base_laser TF comes from the URDF via robot_state_publisher — do NOT use
the static_transform_publisher from the driver's bundled ld19.launch.py.

Usage:
  ros2 launch mark_five_bot lidar.launch.py
  ros2 launch mark_five_bot lidar.launch.py lidar_port:=/dev/ttyUSB1

The argument is named lidar_port (not port) on purpose: launch configurations
are global across included launch files, and serial.launch.py already owns
'port' for the Arduino (/dev/ttyACM0) — reusing the name makes the lidar
open the Arduino's port when both are included from bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of the LD19 CP2102 USB-UART adapter'
    )

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld19_lidar',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': LaunchConfiguration('lidar_port')},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            # Self-hit masking: at 22.5 cm the scan plane can clip the cart's own
            # rear uprights. If fixed close-range points appear behind the robot,
            # set enable_angle_crop_func true and tune the interval (degrees,
            # 0 = lidar forward).
            {'enable_angle_crop_func': True},
            {'angle_crop_min': 110.0},
            {'angle_crop_max': 250.0},
        ],
    )

    return LaunchDescription([
        port_arg,
        ldlidar_node,
    ])
