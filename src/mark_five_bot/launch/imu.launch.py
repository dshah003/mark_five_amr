"""
Launch file for ICM20948 IMU driver + complementary filter

Launches:
  - icm20948_node: I2C driver publishing raw IMU data on /imu/data_raw
  - complementary_filter_node (imu_tools): estimates gyro bias while the
    robot is stationary and publishes bias-corrected data on /imu/data.
    The EKF fuses yaw rate from /imu/data (raw gyro has ~0.015 rad/s
    zero-rate bias that integrates to ~52 deg/min of phantom yaw).

Hardware connection (Jetson Nano):
  ICM20948 VIN → Pin 1 (3.3V)
  ICM20948 GND → Pin 6 (GND)
  ICM20948 SDA → Pin 3 (I2C Bus 1 SDA / GPIO2)
  ICM20948 SCL → Pin 5 (I2C Bus 1 SCL / GPIO3)

I2C Address: 0x68 (default for GY-ICM20948v2 boards)

Usage:
  ros2 launch mark_five_bot imu.launch.py
  ros2 launch mark_five_bot imu.launch.py i2c_address:=0x69  # If using alternate address
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='104',  # 0x68 in decimal
        description='I2C address of ICM20948 (104=0x68 or 105=0x69)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='30',
        description='Publishing rate in Hz (matches EKF frequency)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ICM20948 IMU driver node (custom smbus2-based driver, Jetson Nano compatible)
    imu_node = Node(
        package='mark_five_icm20948',
        executable='icm20948_node',
        name='icm20948_node',
        output='screen',
        parameters=[{
            'i2c_address': LaunchConfiguration('i2c_address'),
            'frame_id': LaunchConfiguration('frame_id'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Complementary filter (imu_tools): subscribes /imu/data_raw, publishes /imu/data.
    # do_bias_estimation learns the gyro zero-rate bias whenever the robot is
    # stationary and subtracts it from the republished angular velocities —
    # this is what makes the gyro usable for EKF yaw-rate fusion.
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        parameters=[{
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'use_mag': False,       # magnetometer useless indoors (motors, rebar)
            'gain_acc': 0.01,
            'bias_alpha': 0.01,
            'publish_tf': False,    # TF comes from URDF via robot_state_publisher
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        i2c_address_arg,
        frame_id_arg,
        pub_rate_arg,
        use_sim_time_arg,
        imu_node,
        imu_filter_node
    ])
