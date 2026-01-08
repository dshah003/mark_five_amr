"""
Launch file for robot_localization EKF node
Fuses wheel odometry (and IMU when available) for improved pose estimation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mark_five_bot')

    # EKF config file
    ekf_config = os.path.join(pkg_dir, 'config', 'robot_localization.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node
    ])
