"""
Odometry Node Launch File

Computes odometry from encoder ticks and publishes /odom + TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('mark_five_bot')

    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'odometry.yaml')

    # Declare launch arguments
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.14',
        description='Distance between wheels in meters'
    )

    ticks_per_meter_arg = DeclareLaunchArgument(
        'ticks_per_meter',
        default_value='3125.0',
        description='Encoder ticks per meter traveled'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish odom->base_footprint TF'
    )

    # Odometry node
    odometry_node = Node(
        package='mark_five_bot',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        wheel_base_arg,
        ticks_per_meter_arg,
        publish_tf_arg,
        odometry_node,
    ])
