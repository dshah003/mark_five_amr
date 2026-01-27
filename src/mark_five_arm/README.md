# Mark Five Arm - ROS2 Robotic Arm Controller

ROS2 package for controlling a hobby servo-based robotic arm using the PCA9685 PWM driver board.

## Overview

This package provides a ROS2 node that interfaces with a PCA9685 16-channel PWM driver to control hobby servos for a robotic arm. It converts joint position commands (in radians) to PWM signals and provides services for homing and relaxing the arm.

### Features

- **ROS2 Jazzy** compatible
- **PCA9685 I2C interface** for controlling up to 16 servos
- **Configurable joint parameters** via YAML files (angle ranges, PWM limits, home positions)
- **ROS2 services** for homing and relaxing the arm
- **Joint state publishing** for monitoring current positions
- **Simulation mode** for testing without hardware
- **GUI testing tool** with joint sliders for manual control

## Hardware Requirements

### Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **PCA9685** | 16-channel 12-bit PWM driver | Servo control via I2C |
| **Hobby Servos** | 4.8-6V, standard PWM (1000-2000μs) | Arm joints |
| **Power Supply** | 5-6V, ≥2A per servo | Servo power |
| **Jetson Nano** | Or similar SBC with I2C | Robot computer |
| **Jumper Wires** | Female-to-female | I2C connections |

### Recommended Servos

For a typical 5-6 DOF arm:
- **Base rotation:** MG996R or SG90 (depending on torque needs)
- **Shoulder/Elbow:** MG996R or DS3218 (high torque)
- **Wrist joints:** SG90 or MG90S (lighter)
- **Gripper:** SG90 with gripper attachment

### uArm Swift Pro (Example Configuration)

If using a **uArm Swift Pro** or similar commercial arm:
- Base: 180° rotation
- Shoulder: 0-90° range
- Elbow: 0-90° range
- Wrist: 180° rotation
- Gripper: Open/close (0-60°)

You'll need to replace the original controller with the PCA9685 and rewire the servos.

## Circuit Diagram

### PCA9685 to Jetson Nano I2C Connection

```
┌─────────────────┐              ┌─────────────────────┐
│  Jetson Nano    │              │     PCA9685         │
│  (GPIO Header)  │              │   PWM Driver        │
├─────────────────┤              ├─────────────────────┤
│                 │              │                     │
│  Pin 1 (3.3V) ──┼─────────────▶│ VCC                 │
│  Pin 3 (SDA)  ──┼─────────────▶│ SDA  [CH0]  [CH8]   │
│  Pin 5 (SCL)  ──┼─────────────▶│ SCL  [CH1]  [CH9]   │
│  Pin 6 (GND)  ──┼─────────────▶│ GND  [CH2]  [CH10]  │
│                 │              │      [CH3]  [CH11]  │
│                 │              │      [CH4]  [CH12]  │
│                 │              │      [CH5]  [CH13]  │
│                 │              │      [CH6]  [CH14]  │
│                 │              │      [CH7]  [CH15]  │
│                 │              │                     │
│                 │              │  V+  ───────────┐   │
│                 │              │  GND ─────────┐ │   │
└─────────────────┘              └───────────────┼─┼───┘
                                                 │ │
                                                 │ │
                                   ┌─────────────┘ │
                                   │  ┌────────────┘
                                   ▼  ▼
                               ┌──────────────┐
                               │ Power Supply │
                               │   5-6V DC    │
                               │   ≥2A/servo  │
                               └──────────────┘
```

### Servo Connections

Each servo connects to a PCA9685 channel:

```
Servo Wire Colors (Standard):
  - Red/Orange: Power (V+) ──▶ PCA9685 V+ terminal
  - Brown/Black: Ground (GND) ──▶ PCA9685 GND terminal
  - Yellow/White: Signal (PWM) ──▶ PCA9685 channel pin

Example 6-DOF Arm:
  CH0: Base rotation servo
  CH1: Shoulder servo
  CH2: Elbow servo
  CH3: Wrist pitch servo
  CH4: Wrist roll servo
  CH5: Gripper servo
```

### Important Wiring Notes

1. **DO NOT power servos from Jetson's 5V pin** - insufficient current (≥2A per servo needed)
2. **Use separate 5-6V power supply** for servo power (V+/GND terminals on PCA9685)
3. **Common ground** - Connect Jetson GND, PCA9685 GND, and power supply GND together
4. **I2C address** - Default PCA9685 address is 0x40 (verify with `i2cdetect`)
5. **I2C bus sharing** - PCA9685 shares I2C bus 1 with IMU (if installed)

## Software Setup

### 1. Build the Package

```bash
# Navigate to workspace
cd ~/mark_five_amr

# Rebuild Docker image with PCA9685 dependencies
cd docker
./build.sh

# Start container
./start.sh
./bash.sh

# Inside container, build the workspace
cd ~/mark_five_amr
colcon build --packages-select mark_five_arm
source install/setup.bash
```

### 2. Verify I2C Connection

Before running the node, verify the PCA9685 is detected:

```bash
# On Jetson host (not Docker)
sudo apt install i2c-tools
sudo i2cdetect -y -r 1

# Expected output:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  ← PCA9685
# ...
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --  ← IMU (if present)
```

If not detected:
- Check wiring (SDA, SCL, VCC, GND)
- Verify PCA9685 power LED is on
- Check I2C address jumpers on PCA9685 board (default 0x40)

### 3. Configure Servo Parameters

Edit `config/arm_params.yaml` to match your arm:

```yaml
# For each joint, configure:
base:
  channel: 0           # PCA9685 channel (0-15)
  min_angle: -1.5708   # Minimum angle in radians
  max_angle: 1.5708    # Maximum angle in radians
  min_pwm: 1000        # PWM value at min_angle (0-4095)
  max_pwm: 2000        # PWM value at max_angle (0-4095)
  home_angle: 0.0      # Home position in radians
  inverted: false      # Reverse direction if true
```

## Calibration

### Step 1: Find PWM Range for Each Servo

Use this test script to find the correct PWM values:

```bash
# Terminal 1: Run arm controller
ros2 launch mark_five_arm arm.launch.py

# Terminal 2: Test PWM values (substitute joint name and angle)
ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base'], position: [0.0]}"

# Try different angles to find limits:
# Min angle: servo at minimum physical position
# Max angle: servo at maximum physical position
# Adjust min_pwm and max_pwm in config until servo moves correctly
```

### Step 2: Calibration Process

For each servo:

1. **Set servo to center position:**
   ```bash
   ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
     "{name: ['base'], position: [0.0]}"
   ```

2. **Find minimum PWM:**
   - Start with `min_pwm: 1000`
   - Decrease until servo reaches minimum physical limit
   - Record this PWM value

3. **Find maximum PWM:**
   - Start with `max_pwm: 2000`
   - Increase until servo reaches maximum physical limit
   - Record this PWM value

4. **Set angle ranges:**
   - Measure physical rotation range (use protractor or known design)
   - Convert to radians: `radians = degrees × π / 180`
   - Update `min_angle` and `max_angle`

5. **Test direction:**
   - Command positive angle
   - If servo moves wrong direction, set `inverted: true`

6. **Set home position:**
   - Choose neutral/safe position (usually 0.0 radians)
   - Verify arm doesn't collide with itself at home

### Step 3: Verify Calibration

```bash
# Launch with GUI sliders
ros2 launch mark_five_arm arm_test.launch.py

# Use sliders to test each joint through full range
# Verify smooth motion and correct angle correspondence
```

### Example Calibration Results

```yaml
# Example: MG996R servo with 180° rotation
base:
  channel: 0
  min_angle: -1.5708   # -90°
  max_angle: 1.5708    # +90°
  min_pwm: 500         # Servo at -90° (found via testing)
  max_pwm: 2400        # Servo at +90° (found via testing)
  home_angle: 0.0      # Center position
  inverted: false

# Example: SG90 servo with 180° rotation
gripper:
  channel: 5
  min_angle: 0.0       # Fully open
  max_angle: 1.0472    # 60° (closed)
  min_pwm: 1000        # Open position
  max_pwm: 2000        # Closed position
  home_angle: 0.5236   # 30° (half-open)
  inverted: false
```

## Usage

### Basic Launch

```bash
# Start arm controller
ros2 launch mark_five_arm arm.launch.py

# In another terminal, command joints
ros2 topic pub /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base', 'shoulder'], position: [0.5, -0.3]}"
```

### Testing with GUI

```bash
# Launch with joint sliders
ros2 launch mark_five_arm arm_test.launch.py

# Or without RViz
ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false
```

Use the GUI sliders to manually control each joint.

### Services

```bash
# Move to home position
ros2 service call /arm/home std_srvs/srv/Trigger

# Relax all servos (disable PWM)
ros2 service call /arm/relax std_srvs/srv/Trigger
```

### Monitor Joint States

```bash
# View current joint positions
ros2 topic echo /arm/joint_states

# Check publishing rate
ros2 topic hz /arm/joint_states
```

## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm/joint_commands` | `sensor_msgs/JointState` | Target joint positions (radians) |

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/arm/joint_states` | `sensor_msgs/JointState` | 10 Hz | Current commanded positions |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/arm/home` | `std_srvs/Trigger` | Move arm to home position |
| `/arm/relax` | `std_srvs/Trigger` | Disable all servos (zero PWM) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_address` | int | 0x40 | PCA9685 I2C address |
| `pwm_frequency` | int | 50 | PWM frequency (Hz) |
| `num_joints` | int | 6 | Number of servo joints |
| `joint_names` | list | ['base', ...] | Joint name list |
| `publish_rate` | float | 10.0 | Joint state publish rate (Hz) |

Per-joint parameters (replace `<joint_name>`):
- `<joint_name>.channel` (int): PCA9685 channel (0-15)
- `<joint_name>.min_angle` (float): Minimum angle (radians)
- `<joint_name>.max_angle` (float): Maximum angle (radians)
- `<joint_name>.min_pwm` (int): PWM at min_angle (0-4095)
- `<joint_name>.max_pwm` (int): PWM at max_angle (0-4095)
- `<joint_name>.home_angle` (float): Home position (radians)
- `<joint_name>.inverted` (bool): Reverse servo direction

## Troubleshooting

### PCA9685 Not Detected

**Problem:** `i2cdetect` doesn't show device at 0x40

**Solutions:**
- Check wiring: VCC, GND, SDA, SCL
- Verify PCA9685 power LED is on
- Try different I2C address (check jumpers on PCA9685 board)
- Test with different I2C bus: `i2cdetect -y -r 0`

### Servo Jitter or Erratic Movement

**Problem:** Servo shakes or moves erratically

**Solutions:**
- Check power supply capacity (≥2A per servo)
- Use thicker power wires to PCA9685 V+ terminal
- Add 100-1000μF capacitor across V+/GND near PCA9685
- Ensure common ground between Jetson, PCA9685, and power supply
- Check for loose connections

### Servo Doesn't Move

**Problem:** No movement when commanding joint

**Solutions:**
- Verify servo is powered (check V+ and GND connections)
- Test PWM with oscilloscope (should see 50Hz signal)
- Check channel number in config matches physical connection
- Try commanding extreme angles: `min_angle` and `max_angle`
- Verify PWM range (1000-2000 is typical, but some servos need 500-2500)

### Wrong Direction

**Problem:** Servo moves opposite to commanded direction

**Solution:**
Set `inverted: true` for that joint in `config/arm_params.yaml`

### Limited Range of Motion

**Problem:** Servo doesn't reach full physical range

**Solutions:**
- Increase PWM range (try `min_pwm: 500`, `max_pwm: 2500`)
- Check servo datasheet for actual PWM range
- Verify no mechanical obstruction

### Node Crashes on Startup

**Problem:** `Failed to initialize PCA9685`

**Solutions:**
- Install Python library: `pip3 install adafruit-circuitpython-pca9685`
- Verify I2C permissions: Add user to `i2c` group
- Run in simulation mode: Node will start without hardware for testing

## Integration with MoveIt2 (Future)

This package provides low-level servo control. For trajectory planning and inverse kinematics:

1. Create URDF model of your arm
2. Configure MoveIt2 with `moveit_setup_assistant`
3. Create hardware interface that publishes to `/arm/joint_commands`
4. Use MoveIt2 for motion planning and collision avoidance

## Files and Directories

```
mark_five_arm/
├── mark_five_arm/
│   ├── __init__.py
│   └── arm_controller.py        # Main PCA9685 controller node
├── config/
│   └── arm_params.yaml          # Servo configuration
├── launch/
│   ├── arm.launch.py            # Basic launch file
│   └── arm_test.launch.py       # Launch with GUI for testing
├── rviz/
│   └── arm.rviz                 # RViz configuration
├── urdf/
│   └── (your arm URDF here)     # Robot description (future)
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
└── README.md                    # This file
```

## Contributing

When adding new features:
1. Follow ROS2 best practices
2. Update configuration files
3. Document parameters and topics
4. Test with simulation mode first

## License

MIT License

## References

- [PCA9685 Datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- [Adafruit PCA9685 Python Library](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

## Support

For issues or questions:
- GitHub Issues: [mark_five_amr repository]
- ROS Answers: Tag with `mark-five-arm`
