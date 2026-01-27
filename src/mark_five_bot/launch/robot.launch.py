"""
Robot Launch File (for Jetson Nano in distributed mode)

Launches sensor and actuation nodes on the robot:
- robot_state_publisher (URDF TF)
- serial_bridge (Arduino communication for motors)
- odometry (encoder-based odometry with per-wheel calibration)
- IMU (ICM-20948 9-DOF sensor)
- EKF filter (sensor fusion - odometry + IMU)
- RealSense camera (with mode switching for laser/visual SLAM)

The workstation runs SLAM, Nav2, and RViz.

Usage:
  # Laser SLAM mode (424x240@15fps - default)
  ros2 launch mark_five_bot robot.launch.py

  # Visual SLAM mode (640x480@30fps - for RTAB-Map)
  ros2 launch mark_five_bot robot.launch.py camera_mode:=visual_slam

  # Without camera
  ros2 launch mark_five_bot robot.launch.py camera:=false

  # Without IMU
  ros2 launch mark_five_bot robot.launch.py imu:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('mark_five_bot')
    description_share = get_package_share_directory('mark_five_description')

    # Path to URDF
    urdf_file = os.path.join(description_share, 'urdf', 'robot.urdf')

    # Declare launch arguments
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='true',
        description='Launch RealSense camera'
    )

    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='laser_scan',
        description='Camera mode: laser_scan (424x240@15fps) or visual_slam (640x480@30fps)'
    )

    imu_arg = DeclareLaunchArgument(
        'imu',
        default_value='true',
        description='Launch ICM-20948 IMU sensor'
    )

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
    )

    # Serial communication with Arduino
    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'serial.launch.py')
        ),
    )

    # Odometry
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'odometry.launch.py')
        ),
    )

    # IMU (ICM-20948 9-DOF sensor - conditional)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'imu.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('imu')),
    )

    # EKF for sensor fusion (fuses wheel odometry + IMU)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ekf.launch.py')
        ),
    )

    # Camera (conditional)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('camera')),
        launch_arguments={
            'mode': LaunchConfiguration('camera_mode'),
        }.items(),
    )

    return LaunchDescription([
        camera_arg,
        camera_mode_arg,
        imu_arg,
        robot_state_publisher_node,
        serial_launch,
        odometry_launch,
        imu_launch,
        ekf_launch,
        camera_launch,
    ])
