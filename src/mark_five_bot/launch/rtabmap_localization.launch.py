"""
RTAB-Map Localization Launch File

Convenience wrapper for launching RTAB-Map in localization-only mode.
Uses existing map database without creating new map data.

Equivalent to: ros2 launch mark_five_bot rtabmap_slam.launch.py localization:=true

Usage:
  ros2 launch mark_five_bot rtabmap_localization.launch.py

  # With custom database path
  ros2 launch mark_five_bot rtabmap_localization.launch.py \
      database_path:=/path/to/rtabmap.db

Features:
- Localization only (no new mapping)
- Loads full map from database on startup
- Publishes robot pose on existing map
- Compatible with Nav2 autonomous navigation

Requirements:
- Existing RTAB-Map database (created during mapping phase)
- Camera and odometry topics active
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('mark_five_bot')

    # Launch arguments
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to existing RTAB-Map database file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Include RTAB-Map SLAM launch file in localization mode
    rtabmap_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rtabmap_slam.launch.py')
        ),
        launch_arguments={
            'localization': 'true',  # KEY: Enable localization mode
            'database_path': LaunchConfiguration('database_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'delete_db_on_start': 'false',  # Don't delete existing map
        }.items(),
    )

    return LaunchDescription([
        database_path_arg,
        use_sim_time_arg,
        rtabmap_slam_launch,
    ])
