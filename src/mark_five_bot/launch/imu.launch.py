"""
Launch file for BNO085 IMU driver

Launches the bno08x_driver node for I2C communication with BNO085 9-DOF IMU.
The IMU provides orientation, angular velocity, and linear acceleration data
for sensor fusion with wheel odometry.

Hardware connection (Jetson Nano):
  BNO085 VIN → Pin 1 (3.3V)
  BNO085 GND → Pin 6 (GND)
  BNO085 SDA → Pin 3 (I2C Bus 1 SDA / GPIO2)
  BNO085 SCL → Pin 5 (I2C Bus 1 SCL / GPIO3)

Usage:
  ros2 launch mark_five_bot imu.launch.py
  ros2 launch mark_five_bot imu.launch.py i2c_bus:=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (Jetson Nano uses bus 1)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # BNO085 IMU driver node
    imu_node = Node(
        package='bno08x_driver',
        executable='bno08x_driver_node',
        name='imu',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('imu/data', 'imu/data')
        ]
    )

    return LaunchDescription([
        i2c_bus_arg,
        use_sim_time_arg,
        imu_node
    ])
