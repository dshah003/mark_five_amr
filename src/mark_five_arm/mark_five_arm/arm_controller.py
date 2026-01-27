#!/usr/bin/env python3
"""
ROS2 Node for controlling robotic arm via PCA9685 PWM driver.

This node subscribes to joint position commands and converts them to PWM signals
for hobby servos connected to a PCA9685 I2C PWM driver board.

Subscribed Topics:
    /arm/joint_commands (sensor_msgs/JointState): Target joint positions in radians

Published Topics:
    /arm/joint_states (sensor_msgs/JointState): Current commanded joint positions

Services:
    /arm/home (std_srvs/Trigger): Move arm to home position
    /arm/relax (std_srvs/Trigger): Disable all servos (zero PWM)

Parameters:
    i2c_address (int): PCA9685 I2C address (default: 0x40)
    pwm_frequency (int): PWM frequency in Hz (default: 50)
    num_joints (int): Number of servo joints (default: 6)
    joint_names (list): Names of joints (default: ['joint_1', ..., 'joint_6'])

    For each joint:
        <joint_name>.min_angle (float): Minimum angle in radians
        <joint_name>.max_angle (float): Maximum angle in radians
        <joint_name>.min_pwm (int): PWM value at min_angle (0-4095)
        <joint_name>.max_pwm (int): PWM value at max_angle (0-4095)
        <joint_name>.home_angle (float): Home position in radians
        <joint_name>.channel (int): PCA9685 channel number (0-15)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import math

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    PCA9685_AVAILABLE = True
except ImportError:
    PCA9685_AVAILABLE = False


class ArmController(Node):
    """ROS2 node for controlling robotic arm servos via PCA9685."""

    def __init__(self):
        super().__init__('arm_controller')

        # Check if PCA9685 library is available
        if not PCA9685_AVAILABLE:
            self.get_logger().error(
                'Adafruit PCA9685 library not found! '
                'Install with: pip3 install adafruit-circuitpython-pca9685'
            )
            raise ImportError('adafruit_pca9685 library required')

        # Declare parameters
        self.declare_parameters()

        # Load parameters
        self.load_parameters()

        # Initialize I2C and PCA9685
        try:
            self.initialize_pca9685()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            self.get_logger().warn('Running in simulation mode (no hardware)')
            self.simulation_mode = True

        # Create subscribers
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/arm/joint_commands',
            self.joint_command_callback,
            10
        )

        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/arm/joint_states',
            10
        )

        # Create services
        self.home_service = self.create_service(
            Trigger,
            '/arm/home',
            self.home_callback
        )

        self.relax_service = self.create_service(
            Trigger,
            '/arm/relax',
            self.relax_callback
        )

        # Initialize joint state message
        self.current_joint_state = JointState()
        self.current_joint_state.name = self.joint_names
        self.current_joint_state.position = [0.0] * self.num_joints

        # Timer for publishing joint states
        self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        self.get_logger().info(f'Arm controller initialized with {self.num_joints} joints')
        self.get_logger().info(f'Joint names: {self.joint_names}')
        if self.simulation_mode:
            self.get_logger().warn('Running in SIMULATION MODE (no PCA9685 hardware)')

        # Move to home position on startup
        self.move_to_home()

    def declare_parameters(self):
        """Declare all ROS2 parameters."""
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('joint_names',
            ['base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper'])
        self.declare_parameter('publish_rate', 10.0)

    def load_parameters(self):
        """Load parameters from ROS2 parameter server."""
        self.i2c_address = self.get_parameter('i2c_address').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.num_joints = self.get_parameter('num_joints').value
        self.joint_names = self.get_parameter('joint_names').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Load per-joint parameters
        self.joint_configs = {}
        for joint_name in self.joint_names:
            # Declare parameters for this joint
            self.declare_parameter(f'{joint_name}.channel', 0)
            self.declare_parameter(f'{joint_name}.min_angle', -math.pi/2)
            self.declare_parameter(f'{joint_name}.max_angle', math.pi/2)
            self.declare_parameter(f'{joint_name}.min_pwm', 1000)
            self.declare_parameter(f'{joint_name}.max_pwm', 2000)
            self.declare_parameter(f'{joint_name}.home_angle', 0.0)
            self.declare_parameter(f'{joint_name}.inverted', False)

            # Load parameters
            self.joint_configs[joint_name] = {
                'channel': self.get_parameter(f'{joint_name}.channel').value,
                'min_angle': self.get_parameter(f'{joint_name}.min_angle').value,
                'max_angle': self.get_parameter(f'{joint_name}.max_angle').value,
                'min_pwm': self.get_parameter(f'{joint_name}.min_pwm').value,
                'max_pwm': self.get_parameter(f'{joint_name}.max_pwm').value,
                'home_angle': self.get_parameter(f'{joint_name}.home_angle').value,
                'inverted': self.get_parameter(f'{joint_name}.inverted').value,
            }

    def initialize_pca9685(self):
        """Initialize I2C bus and PCA9685 driver."""
        self.simulation_mode = False

        # Create I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)

        # Create PCA9685 instance
        self.pca = PCA9685(i2c, address=self.i2c_address)

        # Set PWM frequency (50Hz is standard for servos)
        self.pca.frequency = self.pwm_frequency

        self.get_logger().info(
            f'PCA9685 initialized at address 0x{self.i2c_address:02X}, '
            f'{self.pwm_frequency}Hz'
        )

    def angle_to_pwm(self, angle, joint_name):
        """
        Convert joint angle (radians) to PWM duty cycle value.

        Args:
            angle: Joint angle in radians
            joint_name: Name of the joint

        Returns:
            PWM duty cycle value (0-65535 for 16-bit PWM)
        """
        config = self.joint_configs[joint_name]

        # Clamp angle to valid range
        angle = max(config['min_angle'], min(config['max_angle'], angle))

        # Invert if needed
        if config['inverted']:
            angle = -angle

        # Linear interpolation from angle to PWM
        angle_range = config['max_angle'] - config['min_angle']
        pwm_range = config['max_pwm'] - config['min_pwm']

        # Normalize angle to 0-1 range
        normalized = (angle - config['min_angle']) / angle_range

        # Map to PWM range
        pwm_value = config['min_pwm'] + (normalized * pwm_range)

        # Convert 12-bit PWM value (0-4095) to 16-bit duty cycle (0-65535)
        duty_cycle = int((pwm_value / 4096.0) * 65535)

        return duty_cycle

    def set_servo_position(self, joint_name, angle):
        """
        Set servo position for a specific joint.

        Args:
            joint_name: Name of the joint
            angle: Target angle in radians
        """
        if self.simulation_mode:
            # In simulation mode, just log the command
            self.get_logger().debug(f'SIM: {joint_name} -> {math.degrees(angle):.1f}°')
            return

        config = self.joint_configs[joint_name]
        channel = config['channel']

        # Convert angle to PWM duty cycle
        duty_cycle = self.angle_to_pwm(angle, joint_name)

        # Set PWM output
        try:
            self.pca.channels[channel].duty_cycle = duty_cycle
        except Exception as e:
            self.get_logger().error(f'Failed to set PWM for {joint_name}: {e}')

    def joint_command_callback(self, msg):
        """
        Handle incoming joint position commands.

        Args:
            msg: JointState message with target positions
        """
        if len(msg.name) != len(msg.position):
            self.get_logger().warn('Joint names and positions length mismatch')
            return

        # Update each joint
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_configs:
                self.set_servo_position(name, position)

                # Update current state
                try:
                    idx = self.joint_names.index(name)
                    self.current_joint_state.position[idx] = position
                except ValueError:
                    pass
            else:
                self.get_logger().warn(f'Unknown joint: {name}')

        # Update timestamp
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()

    def publish_joint_states(self):
        """Publish current joint states."""
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(self.current_joint_state)

    def move_to_home(self):
        """Move all joints to home position."""
        self.get_logger().info('Moving to home position')

        for i, joint_name in enumerate(self.joint_names):
            home_angle = self.joint_configs[joint_name]['home_angle']
            self.set_servo_position(joint_name, home_angle)
            self.current_joint_state.position[i] = home_angle

        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()

    def relax_servos(self):
        """Disable all servos by setting PWM to 0."""
        if self.simulation_mode:
            self.get_logger().info('SIM: Relaxing all servos')
            return

        self.get_logger().info('Relaxing all servos')

        for joint_name in self.joint_names:
            channel = self.joint_configs[joint_name]['channel']
            try:
                self.pca.channels[channel].duty_cycle = 0
            except Exception as e:
                self.get_logger().error(f'Failed to relax {joint_name}: {e}')

    def home_callback(self, request, response):
        """Service callback to move arm to home position."""
        try:
            self.move_to_home()
            response.success = True
            response.message = 'Moved to home position'
        except Exception as e:
            response.success = False
            response.message = f'Failed to home: {str(e)}'

        return response

    def relax_callback(self, request, response):
        """Service callback to relax all servos."""
        try:
            self.relax_servos()
            response.success = True
            response.message = 'All servos relaxed'
        except Exception as e:
            response.success = False
            response.message = f'Failed to relax: {str(e)}'

        return response

    def destroy_node(self):
        """Clean up before shutdown."""
        self.get_logger().info('Shutting down arm controller')
        self.relax_servos()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        arm_controller = ArmController()
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
