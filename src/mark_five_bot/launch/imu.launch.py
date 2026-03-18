"""
Launch file for ICM20948 IMU driver

Launches the icm20948_node for I2C communication with ICM-20948 9-DOF IMU.
The IMU provides angular velocity, linear acceleration, and magnetometer data
for sensor fusion with wheel odometry.

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
            'i2c_address': 0x68,  # GY-ICM20948v2 default address (104 in decimal)
            'frame_id': LaunchConfiguration('frame_id'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        i2c_address_arg,
        frame_id_arg,
        pub_rate_arg,
        use_sim_time_arg,
        imu_node
    ])
