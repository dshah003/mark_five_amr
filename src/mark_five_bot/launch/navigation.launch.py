"""
Nav2 Navigation Launch File for Mark Five AMR

Supports two modes:
- slam: Online SLAM with slam_toolbox (for mapping)
- localization: AMCL localization with pre-built map (for navigation)

Usage:
  ros2 launch mark_five_bot navigation.launch.py mode:=slam
  ros2 launch mark_five_bot navigation.launch.py mode:=localization map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mark_five_bot')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    # Default map
    default_map = os.path.join(pkg_share, 'maps', 'map.yaml')

    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode: slam or localization'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file (required for localization mode)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )

    # SLAM mode - includes slam_toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'slam'"])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # Localization mode - includes AMCL + map_server
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    # Nav2 bringup (controller, planner, behaviors, bt_navigator)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        use_sim_time_arg,
        autostart_arg,
        slam_launch,
        localization_launch,
        nav2_bringup_launch,
    ])
