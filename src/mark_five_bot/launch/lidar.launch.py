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
            # Self-hit masking: at 22.5 cm the scan plane clips the cart's own
            # front-center tube pair (measured 2026-07-04: left tube at
            # 106-109 deg, r=0.07 m; right tube mirrored ~251-254 deg). The old
            # 110-250 window missed both by ~1 deg — those in-footprint points
            # made the collision monitor compute collision-time 0 and freeze
            # the robot completely. 100-260 covers them with ~6 deg margin.
            {'enable_angle_crop_func': True},
            {'angle_crop_min': 100.0},
            {'angle_crop_max': 260.0},
        ],
    )

    return LaunchDescription([
        port_arg,
        ldlidar_node,
    ])
