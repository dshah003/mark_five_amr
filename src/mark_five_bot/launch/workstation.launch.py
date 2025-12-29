"""
Workstation Launch File (for remote workstation in distributed mode)

Launches visualization and control nodes:
- teleop (keyboard or joystick)
- RViz (optional visualization)
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

    # Path to RViz config
    rviz_config = os.path.join(description_share, 'rviz', 'display.rviz')

    # Declare launch arguments
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='keyboard',
        description='Teleop mode: keyboard or joy'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
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

    # RViz (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        teleop_arg,
        use_rviz_arg,
        teleop_keyboard_launch,
        teleop_joy_launch,
        rviz_node,
    ])
