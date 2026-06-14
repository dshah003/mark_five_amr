"""
Intel RealSense D435 Camera Launch File

Supports two modes via 'mode' argument:
- laser_scan: 424x240@15fps, optimized for depthimage_to_laserscan (default)
- visual_slam: 640x480@30fps, optimized for RTAB-Map RGB-D SLAM

Launches:
- RealSense camera driver (realsense2_camera)
- depthimage_to_laserscan converter (only in laser_scan mode)

Topics published (all modes):
- /camera/color/image_raw       : RGB image
- /camera/depth/image_rect_raw  : Depth image
- /camera/aligned_depth_to_color/image_raw : Aligned depth (visual_slam mode)

Topics published (laser_scan mode only):
- /scan                         : 2D laser scan (for navigation)

Usage:
  ros2 launch mark_five_bot camera.launch.py mode:=laser_scan   # Default
  ros2 launch mark_five_bot camera.launch.py mode:=visual_slam  # For RTAB-Map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('mark_five_bot')
    realsense_share = get_package_share_directory('realsense2_camera')

    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='laser_scan',
        description='Camera mode: laser_scan (424x240@15fps) or visual_slam (640x480@30fps)'
    )

    # Include RealSense camera launch with mode-dependent parameters
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'initial_reset': 'true',
            'rotation_filter.enable': 'true',
            'rotation_filter.rotation': '180',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # Point cloud: disabled in visual_slam mode (RTAB-Map generates its own)
            'pointcloud.enable': PythonExpression([
                "'false' if '", LaunchConfiguration('mode'), "' == 'visual_slam' else 'true'"
            ]),
            # Resolution: 640x480@15fps for visual_slam (30fps causes depth stream failure on Jetson Nano),
            # 424x240@15fps for laser_scan
            'rgb_camera.color_profile': PythonExpression([
                "'640x480x15' if '", LaunchConfiguration('mode'), "' == 'visual_slam' else '424x240x15'"
            ]),
            'depth_module.depth_profile': PythonExpression([
                "'640x480x15' if '", LaunchConfiguration('mode'), "' == 'visual_slam' else '424x240x15'"
            ]),
            # Align depth: required for visual_slam, optional for laser_scan
            'align_depth.enable': 'true',  # Always enabled (needed for both modes)
            # Decimation: disabled for visual_slam (preserves resolution), enabled for laser_scan
            'decimation_filter.enable': PythonExpression([
                "'false' if '", LaunchConfiguration('mode'), "' == 'visual_slam' else 'true'"
            ]),
            # Spatial/temporal filters: always enabled for noise reduction
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            # Hole filling: enabled for visual_slam, disabled for laser_scan
            'hole_filling_filter.enable': PythonExpression([
                "'true' if '", LaunchConfiguration('mode'), "' == 'visual_slam' else 'false'"
            ]),
        }.items(),
    )

    # Depth to laserscan converter (always enabled)
    # Required in both modes: collision_monitor on workstation needs /scan regardless of SLAM mode
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame': 'camera_link',
            'scan_height': 1,
            'range_min': 0.15,
            'range_max': 4.0,
            'scan_time': 0.0667,
            'depth_scale': 0.001,
        }],
        remappings=[
            ('depth', '/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        mode_arg,
        realsense_launch,
        depth_to_scan_node,
    ])
