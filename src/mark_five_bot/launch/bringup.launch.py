"""
Mark Five AMR Full Bringup Launch File

Launches all core nodes:
- micro-ROS agent (Arduino communication)
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

    return LaunchDescription([
        use_teleop_arg,
        use_camera_arg,
        robot_state_publisher_node,
        serial_launch,
        odometry_launch,
        teleop_launch,
        camera_launch,
    ])
