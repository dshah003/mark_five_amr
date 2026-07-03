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
        self.gyro_bias = self._calibrate_gyro_bias()

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

        # Bank 2: full scale + digital low-pass filter.
        # FCHOICE (bit 0) must be 1 or the DLPF is bypassed and the gyro streams
        # at ~12 kHz bandwidth — sample-to-sample jumps of 0.02–0.05 rad/s, which
        # defeats imu_complementary_filter's steady-state detector (its bias
        # estimator requires consecutive deltas < 0.01 rad/s) and leaves a
        # residual bias that integrates into yaw drift.
        # Gyro:  ±250 dps (131 LSB/dps), DLPF cfg 5 (11.6 Hz 3dB — plenty for a
        #        robot turning ≤ ~60 dps, and matched to the 30 Hz publish rate)
        #        -> (5 << 3) | (0b00 << 1) | 1 = 0x29
        # Accel: ±4g (8192 LSB/g), DLPF cfg 5 (11.5 Hz)
        #        -> (5 << 3) | (0b01 << 1) | 1 = 0x2B
        self._set_bank(2)
        self.bus.write_byte_data(self.address, GYRO_CONFIG_1, 0x29)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x2B)

        self._set_bank(0)

    def _read_gyro_rads(self):
        """Read one gyro sample, returned in rad/s (±250 dps: 131 LSB/dps)."""
        data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 12)
        _, _, _, gx, gy, gz = struct.unpack('>hhhhhh', bytes(data))
        k = math.pi / (131.0 * 180.0)
        return gx * k, gy * k, gz * k

    def _calibrate_gyro_bias(self, n_samples=64, rate_hz=30.0):
        """Average ~2 s of stationary samples to measure gyro zero-rate bias.

        The robot must be stationary during bringup (it always is — motors are
        idle until a teleop/Nav2 command arrives). The constant bias is
        subtracted from every published sample; imu_complementary_filter then
        only has to track the slow thermal drift on top.
        """
        self.get_logger().info(
            f'Calibrating gyro bias ({n_samples / rate_hz:.1f}s) — keep robot stationary...'
        )
        sums = [0.0, 0.0, 0.0]
        maxima = [0.0, 0.0, 0.0]
        for _ in range(n_samples):
            sample = self._read_gyro_rads()
            for i in range(3):
                sums[i] += sample[i]
                maxima[i] = max(maxima[i], abs(sample[i]))
            time.sleep(1.0 / rate_hz)
        bias = [s / n_samples for s in sums]
        # ±250 dps + 11.6 Hz DLPF: stationary samples should sit well under
        # 0.05 rad/s. Larger excursions mean the robot moved mid-calibration.
        if any(m > 0.05 for m in maxima):
            self.get_logger().warn(
                f'Gyro moved during calibration (peak {max(maxima):.3f} rad/s) — '
                'bias estimate may be poor; restart the node with the robot still'
            )
        self.get_logger().info(
            f'Gyro bias: x={bias[0]:+.5f} y={bias[1]:+.5f} z={bias[2]:+.5f} rad/s'
        )
        return bias

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

        # ±4g full scale: 8192 LSB/g
        msg.linear_acceleration.x = ax * 9.81 / 8192.0
        msg.linear_acceleration.y = ay * 9.81 / 8192.0
        msg.linear_acceleration.z = az * 9.81 / 8192.0

        # ±250 dps full scale: 131 LSB/dps, startup bias removed
        k = math.pi / (131.0 * 180.0)
        msg.angular_velocity.x = gx * k - self.gyro_bias[0]
        msg.angular_velocity.y = gy * k - self.gyro_bias[1]
        msg.angular_velocity.z = gz * k - self.gyro_bias[2]

        # No orientation estimate from this driver
        msg.orientation_covariance[0] = -1.0

        # ICM-20948 datasheet noise specs (diagonal covariance matrices)
        # Gyro: ~0.015 dps/√Hz → ~0.004 rad/s RMS at 30 Hz
        # Accel: ~230 μg/√Hz → ~0.04 m/s² RMS at 30 Hz
        gv = 4e-5   # (0.006 rad/s)²
        av = 1.6e-3  # (0.04 m/s²)²
        msg.angular_velocity_covariance     = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        msg.linear_acceleration_covariance  = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ICM20948Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
