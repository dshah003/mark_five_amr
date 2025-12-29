#!/usr/bin/env python3
"""
Serial Bridge Node for Mark Five AMR

Bridges serial communication with Arduino to ROS2 topics.
Replaces micro_ros_agent with a simpler, more reliable approach.

Serial Protocol:
  RX (from Arduino): "t,<left_ticks>,<right_ticks>\n"
  TX (to Arduino):   "v,<linear_x>,<angular_z>\n"

ROS2 Topics:
  Publishers:
    /left_ticks  (std_msgs/Int16)
    /right_ticks (std_msgs/Int16)
  Subscribers:
    /cmd_vel (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import serial
import threading


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout', 0.1)

        # Get parameters
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        timeout = self.get_parameter('timeout').value

        # Publishers for encoder ticks
        self.left_ticks_pub = self.create_publisher(Int16, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int16, 'right_ticks', 10)

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Serial connection
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.running = True

        try:
            self.serial_port = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to Arduino on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return

        # Start serial read thread
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        """Send velocity command to Arduino."""
        if self.serial_port is None or not self.serial_port.is_open:
            return

        # Format: "v,<linear_x>,<angular_z>\n"
        cmd = f"v,{msg.linear.x:.4f},{msg.angular.z:.4f}\n"

        with self.serial_lock:
            try:
                self.serial_port.write(cmd.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def serial_read_loop(self):
        """Read and parse serial data from Arduino."""
        while self.running and self.serial_port is not None:
            try:
                if self.serial_port.in_waiting > 0:
                    with self.serial_lock:
                        line = self.serial_port.readline().decode('utf-8').strip()

                    if line.startswith('t,'):
                        self.parse_ticks(line)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            except UnicodeDecodeError:
                pass  # Ignore decode errors

    def parse_ticks(self, line: str):
        """Parse tick message and publish to ROS2 topics."""
        try:
            # Format: "t,<left_ticks>,<right_ticks>"
            parts = line.split(',')
            if len(parts) == 3:
                left_ticks = int(parts[1])
                right_ticks = int(parts[2])

                left_msg = Int16()
                left_msg.data = left_ticks
                self.left_ticks_pub.publish(left_msg)

                right_msg = Int16()
                right_msg.data = right_ticks
                self.right_ticks_pub.publish(right_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse tick data: {line}')

    def destroy_node(self):
        """Clean up on shutdown."""
        self.running = False
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
