#!/usr/bin/env python3
"""
ROS2 IMU Driver Test Script

This script tests the bno08x_driver ROS2 node by subscribing to /imu/data
and displaying orientation, angular velocity, and linear acceleration.

Prerequisites:
1. Hardware connected and detected (run test_imu_i2c.py first)
2. ROS2 workspace built and sourced

Usage:
  # Terminal 1: Launch IMU driver
  ros2 launch mark_five_bot imu.launch.py

  # Terminal 2: Run this test script
  python3 test_imu_ros.py

  # Or run IMU node directly for testing
  ros2 run bno08x_driver bno08x_driver_node --ros-args -p i2c_bus:=1
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import sys


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convert to degrees
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw)
    )


class IMUTest(Node):
    def __init__(self):
        super().__init__('imu_test')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.msg_count = 0
        self.start_time = self.get_clock().now()

        print("\n" + "="*70)
        print("BNO085 IMU ROS2 Driver Test")
        print("="*70)
        print("\nWaiting for IMU data on /imu/data topic...")
        print("(Press Ctrl+C to stop)\n")
        print("="*70)

    def imu_callback(self, msg):
        self.msg_count += 1

        # Convert quaternion to euler angles
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # Calculate update rate
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        rate = self.msg_count / elapsed if elapsed > 0 else 0

        # Clear screen and print header (every message)
        print("\033[2J\033[H")  # Clear screen and move cursor to top
        print("="*70)
        print(f"BNO085 IMU Data | Messages: {self.msg_count} | Rate: {rate:.1f} Hz")
        print("="*70)

        # Orientation (from quaternion)
        print("\n📐 ORIENTATION (Euler Angles)")
        print(f"  Roll:  {roll:>8.2f}° (rotation around X-axis)")
        print(f"  Pitch: {pitch:>8.2f}° (rotation around Y-axis)")
        print(f"  Yaw:   {yaw:>8.2f}° (rotation around Z-axis)")

        # Orientation (raw quaternion)
        print("\n🔢 ORIENTATION (Quaternion)")
        print(f"  x: {msg.orientation.x:>8.4f}")
        print(f"  y: {msg.orientation.y:>8.4f}")
        print(f"  z: {msg.orientation.z:>8.4f}")
        print(f"  w: {msg.orientation.w:>8.4f}")

        # Angular velocity (gyroscope)
        print("\n🌀 ANGULAR VELOCITY (rad/s)")
        print(f"  x: {msg.angular_velocity.x:>8.4f} rad/s ({math.degrees(msg.angular_velocity.x):>7.2f}°/s)")
        print(f"  y: {msg.angular_velocity.y:>8.4f} rad/s ({math.degrees(msg.angular_velocity.y):>7.2f}°/s)")
        print(f"  z: {msg.angular_velocity.z:>8.4f} rad/s ({math.degrees(msg.angular_velocity.z):>7.2f}°/s)")

        # Linear acceleration (accelerometer)
        print("\n🚀 LINEAR ACCELERATION (m/s²)")
        print(f"  x: {msg.linear_acceleration.x:>8.4f} m/s²")
        print(f"  y: {msg.linear_acceleration.y:>8.4f} m/s²")
        print(f"  z: {msg.linear_acceleration.z:>8.4f} m/s² (gravity ≈ 9.81 when upright)")

        # Total acceleration magnitude
        accel_mag = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        print(f"  Magnitude: {accel_mag:.4f} m/s²")

        print("\n" + "="*70)
        print("Press Ctrl+C to stop")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = IMUTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("1. Is the IMU driver running?")
        print("   ros2 launch mark_five_bot imu.launch.py")
        print("2. Check if /imu/data topic exists:")
        print("   ros2 topic list | grep imu")
        print("3. Verify IMU hardware connection:")
        print("   sudo python3 test_imu_i2c.py")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
