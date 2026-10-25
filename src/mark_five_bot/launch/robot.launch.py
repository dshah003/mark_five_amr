"""
Robot Launch File (for Jetson Nano in distributed mode)

Launches sensor and actuation nodes on the robot:
- robot_state_publisher (URDF TF)
- serial_bridge (Arduino communication for motors)
- safety_watchdog (mark_five_safety, use_safety:=true by default) -> /cmd_vel_gated
- odometry (encoder-based odometry with per-wheel calibration)
- IMU (ICM-20948 9-DOF sensor)
- EKF filter (sensor fusion - odometry + IMU)
- RealSense camera (with mode switching for laser/visual SLAM)
- RTAB-Map SLAM (optional, runs locally to avoid WiFi bandwidth bottleneck)

The workstation runs Nav2 and RViz. When slam:=rtab, SLAM also runs here.

Usage:
  # Laser SLAM mode (workstation handles SLAM)
  ros2 launch mark_five_bot robot.launch.py

  # Visual SLAM on Jetson (avoids sending raw RGB-D over WiFi)
  ros2 launch mark_five_bot robot.launch.py camera_mode:=visual_slam slam:=rtab

  # Without camera
  ros2 launch mark_five_bot robot.launch.py camera:=false

  # Without IMU
  ros2 launch mark_five_bot robot.launch.py imu:=false

  # Without the safety watchdog (serial_bridge listens to /cmd_vel_safe directly)
  ros2 launch mark_five_bot robot.launch.py use_safety:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='none',
        description='SLAM to run on Jetson: none or rtab'
    )

    use_safety_arg = DeclareLaunchArgument(
        'use_safety',
        default_value='true',
        description='Launch the mark_five_safety watchdog (S2 heartbeat + S4 velocity caps) '
                    'between /cmd_vel_safe and serial_bridge (/cmd_vel_gated)'
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

    # Safety watchdog (mark_five_safety): last hop before the motors.
    #   /cmd_vel_safe -> safety_watchdog -> /cmd_vel_gated -> serial_bridge
    # use_safety:=false skips it and points serial_bridge back at /cmd_vel_safe,
    # so serial_bridge never listens to a topic nobody publishes. The path is a
    # lazy substitution, so use_safety:=false works even if mark_five_safety
    # isn't built.
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mark_five_safety'), 'launch', 'safety.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('use_safety')),
    )
    serial_cmd_vel_topic = PythonExpression([
        "'/cmd_vel_gated' if '", LaunchConfiguration('use_safety'),
        "'.lower() in ('true', '1') else '/cmd_vel_safe'"
    ])

    # Serial communication with Arduino
    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'serial.launch.py')
        ),
        launch_arguments={'serial_cmd_vel_topic': serial_cmd_vel_topic}.items(),
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

    # RTAB-Map SLAM on Jetson (optional — avoids sending raw RGB-D over WiFi)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rtabmap_slam.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('slam'), "' == 'rtab'"])
        ),
    )

    return LaunchDescription([
        camera_arg,
        camera_mode_arg,
        imu_arg,
        slam_arg,
        use_safety_arg,
        robot_state_publisher_node,
        safety_launch,
        serial_launch,
        odometry_launch,
        imu_launch,
        ekf_launch,
        camera_launch,
        rtabmap_launch,
    ])
