"""
SLAM Launch File for Mark Five AMR

Launches slam_toolbox in online async mode for real-time mapping.

Usage:
  ros2 launch mark_five_bot slam.launch.py

To save map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mark_five_bot')

    # Default params file
    default_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='Full path to slam_toolbox params file'
    )

    # SLAM Toolbox node - online async mode
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
    ])
