"""
Intel RealSense D435 Camera Launch File

Launches:
- RealSense camera driver (realsense2_camera)
- depthimage_to_laserscan converter for navigation

Topics published:
- /camera/color/image_raw       : RGB image
- /camera/depth/image_rect_raw  : Depth image
- /camera/depth/points          : Point cloud
- /scan                         : 2D laser scan (for navigation)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('mark_five_bot')
    realsense_share = get_package_share_directory('realsense2_camera')

    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    # Include RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'pointcloud.enable': 'true',
            'rgb_camera.color_profile': '424x240x15',
            'depth_module.depth_profile': '424x240x15',
            'align_depth.enable': 'true',
            'decimation_filter.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
        }.items(),
    )

    # Depth to laserscan converter
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame': 'camera_link',
            'scan_height': 10,
            'range_min': 0.15,
            'range_max': 4.0,
            'scan_time': 0.0667,
        }],
        remappings=[
            ('image', '/camera/depth/image_rect_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        realsense_launch,
        depth_to_scan_node,
    ])
