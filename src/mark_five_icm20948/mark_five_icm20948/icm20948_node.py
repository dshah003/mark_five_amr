import math
import struct
import time

import rclpy
import sensor_msgs.msg
from rclpy.node import Node
from smbus2 import SMBus

# Register Bank Select
REG_BANK_SEL = 0x7F

# Bank 0 registers
WHO_AM_I     = 0x00
PWR_MGMT_1   = 0x06
PWR_MGMT_2   = 0x07
ACCEL_XOUT_H = 0x2D  # First of 12 bytes: accel xyz + gyro xyz

# Bank 2 registers
GYRO_CONFIG_1 = 0x01
ACCEL_CONFIG  = 0x14

WHO_AM_I_VAL = 0xEA  # Expected WHO_AM_I value for ICM-20948


class ICM20948Node(Node):
    def __init__(self):
        super().__init__('icm20948_node')

        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate', 30)

        self.address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        i2c_bus     = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        pub_rate    = self.get_parameter('pub_rate').get_parameter_value().integer_value

        self.bus = SMBus(i2c_bus)
        self._init_imu()

        self.imu_pub = self.create_publisher(sensor_msgs.msg.Imu, '/imu/data_raw', 10)
        self.create_timer(1.0 / pub_rate, self._publish)

        self.get_logger().info(
            f'ICM-20948 ready (address=0x{self.address:02x}, bus={i2c_bus}, rate={pub_rate}Hz)'
        )

    def _set_bank(self, bank):
        self.bus.write_byte_data(self.address, REG_BANK_SEL, bank << 4)

    def _init_imu(self):
        self._set_bank(0)

        who = self.bus.read_byte_data(self.address, WHO_AM_I)
        if who != WHO_AM_I_VAL:
            self.get_logger().error(
                f'Unexpected WHO_AM_I: 0x{who:02x} (expected 0x{WHO_AM_I_VAL:02x})'
            )

        # Reset, then wake with auto-select clock
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable accel and gyro
        self.bus.write_byte_data(self.address, PWR_MGMT_2, 0x00)

        # Bank 2: set full scale ranges
        # Gyro ±2000 dps (16.4 LSB/dps): bits[2:1] = 11 -> 0x06
        # Accel ±16g (2048 LSB/g):        bits[2:1] = 11 -> 0x06
        self._set_bank(2)
        self.bus.write_byte_data(self.address, GYRO_CONFIG_1, 0x06)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x06)

        self._set_bank(0)

    def _publish(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 12)
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}', throttle_duration_sec=5.0)
            return

        ax, ay, az, gx, gy, gz = struct.unpack('>hhhhhh', bytes(data))

        msg = sensor_msgs.msg.Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ±16g full scale: 2048 LSB/g
        msg.linear_acceleration.x = ax * 9.81 / 2048.0
        msg.linear_acceleration.y = ay * 9.81 / 2048.0
        msg.linear_acceleration.z = az * 9.81 / 2048.0

        # ±2000 dps full scale: 16.4 LSB/dps
        msg.angular_velocity.x = gx * math.pi / (16.4 * 180.0)
        msg.angular_velocity.y = gy * math.pi / (16.4 * 180.0)
        msg.angular_velocity.z = gz * math.pi / (16.4 * 180.0)

        # No orientation estimate from this driver
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ICM20948Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
