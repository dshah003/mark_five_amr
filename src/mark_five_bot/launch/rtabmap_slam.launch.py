"""
RTAB-Map Visual SLAM Launch File for Mark Five AMR

Launches RTAB-Map in RGB-D SLAM mode for real-time visual mapping.
Replaces slam_toolbox for visual-based SLAM using the RealSense D435 camera.

Features:
- RGB-D visual SLAM with loop closure detection
- Publishes /map (occupancy grid) and map→odom TF for Nav2 compatibility
- Database auto-saves to ~/.ros/rtabmap.db
- Supports both mapping and localization modes

Usage:
  # Mapping mode (build new map)
  ros2 launch mark_five_bot rtabmap_slam.launch.py

  # Localization mode (use existing map, no new mapping)
  ros2 launch mark_five_bot rtabmap_slam.launch.py localization:=true

  # Custom database path
  ros2 launch mark_five_bot rtabmap_slam.launch.py database_path:=/path/to/rtabmap.db

  # Delete database and start fresh
  ros2 launch mark_five_bot rtabmap_slam.launch.py delete_db_on_start:=true

Topics Subscribed:
  /camera/color/image_raw              - RGB image from RealSense
  /camera/color/camera_info            - RGB camera calibration
  /camera/aligned_depth_to_color/image_raw - Aligned depth image
  /odometry/filtered                   - EKF-fused odometry (wheel + IMU)

Topics Published:
  /map                                 - Occupancy grid for Nav2
  /rtabmap/cloud_map                   - 3D point cloud map
  /rtabmap/localization_pose           - Localization pose estimate
  /rtabmap/info                        - SLAM statistics

TF Published:
  map → odom                           - SLAM correction transform
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('mark_five_bot')

    # RTAB-Map configuration file
    rtabmap_config = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode (uses existing map, no new mapping)'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTAB-Map database file'
    )

    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Delete database on startup (start fresh map)'
    )

    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map visualization GUI (workstation only)'
    )

    # Parameters for RTAB-Map nodes
    parameters = [{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'database_path': LaunchConfiguration('database_path'),
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,  # Pure visual mode (no laser scan)
        'subscribe_rgbd': True,   # Subscribe to synchronized rgbd_image from rgbd_sync
        'subscribe_scan': True,   # Fuse 2D laser scan for better occupancy grid accuracy
        'approx_sync': True,      # Critical for distributed mode
        'queue_size': 30,
        'qos': 1,
    }]

    # RTAB-Map parameters with localization mode override
    # When localization=true: disable incremental memory, load full map
    rtabmap_parameters = [{
        'Mem/IncrementalMemory': PythonExpression([
            "'false' if '", LaunchConfiguration('localization'), "' == 'true' else 'true'"
        ]),
        'Mem/InitWMWithAllNodes': LaunchConfiguration('localization'),
    }]

    # rgbd_sync node - synchronizes RGB, Depth, and Odometry
    # This is critical for distributed mode where topics may have slight time misalignment
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=parameters,  # Only basic parameters, not full rtabmap_config
        remappings=[
            # Camera inputs
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),

            # Odometry input (EKF-fused wheel + IMU)
            ('odom', '/odometry/filtered'),

            # Synchronized RGB-D output (internal topic to rtabmap node)
            ('rgbd_image', 'rgbd_image'),
        ],
    )

    # RTAB-Map core SLAM node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters + [rtabmap_config] + rtabmap_parameters,
        arguments=[
            PythonExpression([
                "'--delete_db_on_start' if '", LaunchConfiguration('delete_db_on_start'), "' == 'true' else ''"
            ]),
        ],
        remappings=[
            # Odometry input
            ('odom', '/odometry/filtered'),

            # Synchronized RGB-D input from rgbd_sync
            ('rgbd_image', 'rgbd_image'),

            # Laser scan input from depthimage_to_laserscan
            ('scan', '/scan'),

            # Map output (for Nav2)
            ('grid_map', '/map'),

            # TF published: map → odom
        ],
    )

    # RTAB-Map visualization (optional, for debugging on workstation)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters + [rtabmap_config],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('rgbd_image', 'rgbd_image'),
        ],
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        localization_arg,
        database_path_arg,
        delete_db_arg,
        rtabmap_viz_arg,

        # Nodes
        rgbd_sync,
        rtabmap_slam,
        rtabmap_viz,
    ])
