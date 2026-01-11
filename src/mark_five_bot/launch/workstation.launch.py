"""
Workstation Launch File (for remote workstation in distributed mode)

Launches heavy compute nodes on the workstation:
- SLAM (slam_toolbox or RTAB-Map) or Localization (AMCL)
- Nav2 navigation stack
- RViz visualization
- Teleop (optional keyboard or joystick)

The Jetson runs sensors and actuation via robot.launch.py.

SLAM Modes:
- laser: slam_toolbox with 2D laser scan (stable, lower CPU)
- visual: RTAB-Map RGB-D visual SLAM (better map quality, default)
- hybrid: RTAB-Map with RGB-D + laser fusion (best quality)

Usage:
  # Visual SLAM mode (default - best map quality)
  ros2 launch mark_five_bot workstation.launch.py

  # Laser SLAM mode (fallback)
  ros2 launch mark_five_bot workstation.launch.py slam_mode:=laser

  # Hybrid SLAM mode
  ros2 launch mark_five_bot workstation.launch.py slam_mode:=hybrid

  # Localization mode (with existing map)
  ros2 launch mark_five_bot workstation.launch.py nav_mode:=localization map:=/path/to/map.yaml

  # With joystick teleop
  ros2 launch mark_five_bot workstation.launch.py teleop:=joy

  # Without navigation (teleop + RViz only)
  ros2 launch mark_five_bot workstation.launch.py use_nav:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('mark_five_bot')
    description_share = get_package_share_directory('mark_five_description')

    # Path to RViz configs
    nav_rviz_config = os.path.join(description_share, 'rviz', 'navigation.rviz')
    default_rviz_config = os.path.join(description_share, 'rviz', 'display.rviz')

    # Default map
    default_map = os.path.join(pkg_share, 'maps', 'map.yaml')

    # Declare launch arguments
    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='true',
        description='Launch SLAM/Nav2 navigation stack'
    )

    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='slam',
        description='Navigation mode: slam or localization'
    )

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='visual',
        description='SLAM sensor mode: laser (slam_toolbox), visual (RTAB-Map), or hybrid (RTAB-Map+laser)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file (for localization mode)'
    )

    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='none',
        description='Teleop mode: keyboard, joy, or none'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Navigation stack (SLAM or Localization + Nav2)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_nav')),
        launch_arguments={
            'mode': LaunchConfiguration('nav_mode'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Keyboard teleop (conditional)
    teleop_keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'teleop_keyboard.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'keyboard'"])
        ),
    )

    # Joystick teleop (conditional)
    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'teleop_joy.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'joy'"])
        ),
    )

    # RViz with navigation config when nav is enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', nav_rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_nav_arg,
        nav_mode_arg,
        slam_mode_arg,
        map_arg,
        teleop_arg,
        use_rviz_arg,
        use_sim_time_arg,
        navigation_launch,
        teleop_keyboard_launch,
        teleop_joy_launch,
        rviz_node,
    ])
