"""
Nav2 Navigation Launch File for Mark Five AMR

Supports two modes:
- slam: Online SLAM with slam_toolbox (for mapping)
- localization: AMCL localization with pre-built map (for navigation)

Optional mission manager for waypoint navigation (use_mission_manager:=true)

Usage:
  ros2 launch mark_five_bot navigation.launch.py mode:=slam
  ros2 launch mark_five_bot navigation.launch.py mode:=localization map:=/path/to/map.yaml
  ros2 launch mark_five_bot navigation.launch.py mode:=localization use_mission_manager:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mark_five_bot')

    # Config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    mission_manager_params_file = os.path.join(pkg_share, 'config', 'mission_manager.yaml')

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

    use_mission_manager_arg = DeclareLaunchArgument(
        'use_mission_manager',
        default_value='false',
        description='Launch waypoint mission manager'
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

    # Nav2 core nodes (without docking_server)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Mission manager (optional)
    mission_manager = Node(
        package='mark_five_bot',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
        parameters=[mission_manager_params_file],
        condition=IfCondition(LaunchConfiguration('use_mission_manager')),
    )

    # Lifecycle manager for navigation nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'smoother_server',
                'velocity_smoother',
                'collision_monitor',
                'waypoint_follower',
            ],
        }],
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        use_sim_time_arg,
        autostart_arg,
        use_mission_manager_arg,
        slam_launch,
        localization_launch,
        # Nav2 nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        velocity_smoother,
        collision_monitor,
        waypoint_follower,
        mission_manager,
        lifecycle_manager,
    ])
