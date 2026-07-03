"""
Mark Five AMR Full Bringup Launch File

Launches all core nodes:
- serial_bridge (Arduino communication via simple serial protocol)
- robot_state_publisher (URDF TF)
- odometry (encoder-based odometry)
- teleop (optional keyboard control)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('mark_five_bot')
    description_share = get_package_share_directory('mark_five_description')

    # Path to URDF
    urdf_file = os.path.join(description_share, 'urdf', 'robot.urdf')

    # Declare launch arguments
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Launch keyboard teleop (requires separate terminal - run manually instead)'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera'
    )

    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='Launch Nav2 navigation stack'
    )

    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='slam',
        description='Navigation mode: slam or localization'
    )

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='visual',
        description='SLAM sensor mode: laser (slam_toolbox), visual (RTAB-Map), or hybrid'
    )

    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Delete RTAB-Map database on startup (start fresh map)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (for localization mode)'
    )

    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Launch ICM-20948 IMU driver + complementary filter (gyro yaw rate for EKF)'
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

    # IMU driver + complementary filter (bias-corrected gyro for EKF yaw rate)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'imu.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    # EKF for sensor fusion (fuses wheel odometry + IMU when available)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ekf.launch.py')
        ),
    )

    # Keyboard teleop (conditional)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'teleop_keyboard.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    # Camera (conditional)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # Navigation (conditional)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_nav')),
        launch_arguments={
            'mode': LaunchConfiguration('nav_mode'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map': LaunchConfiguration('map'),
            'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
        }.items(),
    )

    return LaunchDescription([
        use_teleop_arg,
        use_camera_arg,
        use_nav_arg,
        nav_mode_arg,
        slam_mode_arg,
        delete_db_arg,
        map_arg,
        use_imu_arg,
        robot_state_publisher_node,
        serial_launch,
        odometry_launch,
        imu_launch,  # IMU driver + bias-correcting filter
        ekf_launch,  # Sensor fusion (odometry + IMU)
        teleop_launch,
        camera_launch,
        navigation_launch,
    ])
