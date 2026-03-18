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

## Supported Arm Designs

This package is configured for the **Lite Arm i2** (Thingiverse thing:480446) - a 4-DOF robotic arm modeled after the uFactory uArm.

**Lite Arm i2 Specifications (Official):**
- **3-4 servos:** 3x Power HD 1501 MG (base, shoulder, elbow) + optional gripper servo
- **4 DOF:** Base rotation (Z-axis), Shoulder (Y-axis), Elbow (Y-axis), Gripper
- **Design:** Inspired by ABB IRB-660 industrial robot
- **3D Printed:** All structural parts are 3D printable
- **STL Files:** Available on [Thingiverse thing:480446](https://www.thingiverse.com/thing:480446)
- **Dimensions (from STL analysis):**
  - Upper arm: 230mm (20_21B.stl)
  - Forearm: 180mm (15C.stl)
  - Base bearing: 35mm ID × 47mm OD × 7mm (6807 2RS)
- **PWM Range (calibrated):**
  - Base: 690-2600μs (center: 1645μs)
  - Shoulder: 900-2000μs (center: 1450μs)
  - Elbow: 900-2350μs (center: 1625μs)

The URDF (`urdf/lite_arm_i2.urdf.xacro`) has been updated with dimensions extracted from the official STL files.

**Arm Configuration (3-DOF, parallel linkage):**
```
       End Effector (passive)
            │
            │ (no servo - parallel linkage keeps it level)
            ▼
      ┌─────────────┐
      │  Forearm    │ ← 180mm (15C.stl)
      │ (2 parallel │
      │   links)    │
      └─────┬───────┘
            │ (revolute, Y-axis)
         Elbow (CH2)
            │
      ┌─────────────┐
      │ Upper Arm   │ ← 230mm (20_21B.stl)
      │ (2 parallel │
      │   links)    │
      └─────┬───────┘
            │ (revolute, Y-axis)
       Shoulder (CH1)
            │
      Rotating Base
            │
            │ (revolute, Z-axis)
        Base (CH0)
            │
         ═════════
        Fixed Base
```

**Note:** The parallel linkage mechanism keeps the end effector/tool at a consistent orientation as the arm moves (similar to ABB IRB-660). The URDF simplifies this to a serial chain for basic visualization. For accurate inverse kinematics, the parallel linkage geometry would need to be modeled.

**Optional Gripper:** Add a gripper servo to channel 3 if needed (see `config/arm_params.yaml`).

## Hardware Requirements

### Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **PCA9685** | 16-channel 12-bit PWM driver | Servo control via I2C |
| **Hobby Servos** | 4.8-6V, standard PWM (1000-2000μs) | Arm joints |
| **Power Supply** | 5-6V, ≥2A per servo | Servo power |
| **Jetson Nano** | Or similar SBC with I2C | Robot computer |
| **Jumper Wires** | Female-to-female | I2C connections |

### Official Servos (Lite Arm i2)

As specified in the official parts list:
- **3x Power HD 1501 MG servos** (base, shoulder, forearm) - $15.95 each
  - Operating voltage: 4.8-6V
  - PWM range: Varies per joint (see calibrated values above)
  - Torque: ~15kg-cm @ 6V
- **Optional gripper servo:** Any standard servo (SG90 or similar)

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

## Measuring and Updating URDF Dimensions

The included URDF has estimated dimensions. For accurate visualization and future kinematics, measure your printed arm and update the URDF.

### What to Measure

Open `urdf/lite_arm_i2.urdf.xacro` and update these properties at the top:

```xml
<!-- Measure these on your printed arm! -->
<xacro:property name="base_height" value="0.050" />      <!-- Base cylinder height -->
<xacro:property name="base_radius" value="0.045" />      <!-- Base cylinder radius -->

<xacro:property name="upper_arm_length" value="0.148" /> <!-- Shoulder to elbow distance -->
<xacro:property name="forearm_length" value="0.160" />   <!-- Elbow to wrist distance -->
<xacro:property name="gripper_length" value="0.080" />   <!-- Gripper reach -->
```

### How to Measure

1. **Base dimensions:**
   - Measure height and radius of the rotating base platform

2. **Upper arm (shoulder to elbow):**
   - Measure center-to-center distance between shoulder servo axis and elbow servo axis
   - This is the most critical dimension for kinematics

3. **Forearm (elbow to wrist/gripper):**
   - Measure center-to-center distance from elbow servo axis to gripper attachment point

4. **Gripper:**
   - Measure from gripper base to tip of fingers when closed

### Verify URDF in RViz

After updating dimensions:

```bash
ros2 launch mark_five_arm display.launch.py
```

Use the joint sliders to move the arm and verify:
- Links are the correct length
- Joints rotate around correct axes
- No unexpected offsets or rotations

## Hardware Assembly

### Parts List

The official Lite Arm i2 parts list is included in `stl/LAi2Partslist.pdf`. Key components:

**Electronics:**
- 3x Power HD 1501 MG servos ($15.95 each)
- 3x 25T servo horns (3mm threaded)
- PCA9685 PWM driver (not in original - added for ROS2 control)

**Mechanical:**
- 24x MF84zz 4x8x3mm flanged ball bearings ($15 for 24)
- 1x 6807 2RS bearing (35mm ID × 47mm OD) for base rotation ($9.47)
- Various 6-32 and 8-32 bolts and nuts (see PDF for full list)

**3D Printed Parts:**
- 30 STL files included in `stl/` directory
- Print all parts in PLA or PETG
- See assembly instructions in `stl/LAi2_Instructions.zip`

### Assembly Notes

1. Follow the official assembly instructions in `LAi2_Instructions.zip`
2. The original design uses direct Arduino control - we're replacing that with PCA9685 for ROS2 integration
3. Wire servos to PCA9685 channels 0-2 (base, shoulder, elbow)
4. Gripper servo (if used) goes to channel 3

## Calibration

### Official PWM Values (Power HD 1501 MG)

The Lite Arm i2 uses **Power HD 1501 MG** servos with these specs (from official Arduino code):

| Position | PWM Value |
|----------|-----------|
| Full left/min | 1000μs |
| **Center** | **1500μs** |
| Full right/max | 2000μs |

These values are **already configured** in `config/arm_params.yaml`. For most builds, no calibration is needed!

### Quick Test (Verify Default Values Work)

```bash
# Terminal 1: Run arm controller
ros2 launch mark_five_arm arm.launch.py

# Terminal 2: Send all servos to center (1500μs)
ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base', 'shoulder', 'elbow'], position: [0.0, 0.0, 0.0]}"
```

All three servos should move to their center positions. If this works, the default calibration is correct!

### When Calibration IS Needed

Only calibrate if:
1. **Servo moves wrong direction** → Set `inverted: true` for that joint
2. **Center position is off** → Adjust PWM values slightly
3. **Using different servos** → Find their PWM range

### Calibration Process (If Needed)

**Step 1: Check servo direction**
```bash
# Command positive angle for base
ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base'], position: [0.5]}"
```
- If base rotates counter-clockwise (looking from above): correct
- If base rotates clockwise: set `base.inverted: true` in config

Repeat for shoulder and elbow.

**Step 2: Verify home position**

The arm should be in a safe "home" pose when all joints are at 0.0:
```bash
ros2 service call /arm/home std_srvs/srv/Trigger
```

If the home position causes collisions or looks wrong, adjust `home_angle` for each joint in the config.

**Step 3: Test full range with GUI**
```bash
ros2 launch mark_five_arm arm_test.launch.py
```

Use the sliders to move each joint through its full range. Verify:
- Smooth motion (no jitter)
- No mechanical binding at limits
- Physical arm matches RViz visualization

### Fine-Tuning PWM (Rare)

If servos don't reach full range or overshoot:

```yaml
# In config/arm_params.yaml
base:
  min_pwm: 1000   # Decrease if servo doesn't reach min position
  max_pwm: 2000   # Increase if servo doesn't reach max position
```

The Power HD 1501 MG servos support extended PWM ranges beyond the nominal 1000-2000μs. Calibrated ranges: base (690-2600μs), shoulder (900-2000μs), elbow (900-2350μs).

### Lite Arm i2 Default Calibration

```yaml
# These values should work out-of-the-box for Lite Arm i2
base:
  channel: 0
  min_angle: -1.5708   # -90°
  max_angle: 1.5708    # +90°
  min_pwm: 1000        # Official spec
  max_pwm: 2000        # Official spec
  home_angle: 0.0      # Center
  inverted: false      # Change if needed
```

## Usage

### 1. Visualize URDF Only (No Hardware)

Visualize the arm model in RViz without connecting to hardware:

```bash
ros2 launch mark_five_arm display.launch.py
```

This launches:
- `robot_state_publisher` - publishes URDF transforms
- `joint_state_publisher_gui` - GUI sliders for joint control
- `rviz2` - visualization

Use this to verify your URDF dimensions before connecting servos.

### 2. Test with Hardware (PCA9685 + Servos)

Test actual servo control with GUI and visualization:

```bash
# Full test with RViz
ros2 launch mark_five_arm arm_test.launch.py

# Or without RViz (faster)
ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false
```

This launches:
- `robot_state_publisher` - publishes URDF transforms
- `arm_controller` - controls PCA9685 hardware
- `joint_state_publisher_gui` - GUI sliders send commands to hardware
- `rviz2` - visualization (optional)

Move the sliders and watch both the physical arm and RViz model move together.

### 2b. Distributed Mode (Jetson + Workstation)

When running on a headless Jetson Nano, run the controller on Jetson and GUI/RViz on your workstation:

**On Jetson (headless):**
```bash
ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false use_gui:=false
```

**On Workstation:**
```bash
# RViz only (view arm state)
ros2 launch mark_five_arm display.launch.py use_gui:=false use_rsp:=false

# GUI control (move arm with sliders)
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -r /joint_states:=/arm/joint_commands
```

Note: `use_rsp:=false` prevents duplicate `robot_state_publisher` since Jetson is already running it.

### 3. Programmatic Control (No GUI)

Control the arm via ROS2 topics:

```bash
# Terminal 1: Start arm controller only
ros2 launch mark_five_arm arm.launch.py

# Terminal 2: Command joints via topic (positions in radians)
ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base', 'shoulder', 'elbow'], position: [0.5, 0.3, -0.5]}"
```

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

**Problem:** `Adafruit PCA9685 library not found`

**Solution:**
```bash
pip3 install --break-system-packages adafruit-circuitpython-pca9685 Jetson.GPIO
```

**Problem:** `module 'board' has no attribute 'SCL'`

**Solution:** Install Jetson GPIO library:
```bash
pip3 install --break-system-packages Jetson.GPIO
```

**Problem:** `Failed to initialize PCA9685`

**Solutions:**
- Verify I2C connection: `sudo i2cdetect -y -r 1` (should show 0x40)
- Check I2C permissions: Add user to `i2c` group
- Node will fall back to simulation mode if hardware not available

### Direct Hardware Test (No ROS2)

Use the test script for debugging without ROS2:
```bash
python3 src/mark_five_arm/scripts/test_servos.py
```
This allows interactive testing of individual servos with direct PWM control.

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
│   └── arm_params.yaml          # Servo configuration (calibrated PWM values)
├── launch/
│   ├── arm.launch.py            # Basic launch (controller only)
│   ├── arm_test.launch.py       # Hardware testing with GUI + RViz
│   └── display.launch.py        # URDF visualization only (no hardware)
├── scripts/
│   └── test_servos.py           # Direct hardware test (no ROS2)
├── rviz/
│   ├── arm.rviz                 # RViz config (legacy)
│   └── display.rviz             # RViz config for URDF display
├── urdf/
│   ├── arm.urdf.xacro           # Generic 6-DOF arm (template)
│   └── lite_arm_i2.urdf.xacro   # Lite Arm i2 specific (4-DOF)
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

### Robotic Arm Design
- [Lite Arm i2 on Thingiverse](https://www.thingiverse.com/thing:480446) - Original 3D printable arm design
- [Lite Arm i2 STL Files](https://www.stlfinder.com/3dmodels/open-source-robotic-arm-lite-arm-i2-mod-files/) - Alternative STL sources
- [Lite Arm i1 (Original)](https://www.thingiverse.com/thing:407800) - Earlier version

### Electronics & Software
- [PCA9685 Datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- [Adafruit PCA9685 Python Library](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

### Related Projects
- [Top 10 Open Source Robotic Arms](https://circuitdigest.com/articles/top-10-opensource-robotic-arms-for-beginners) - Comparison of DIY arm designs

## Support

For issues or questions:
- GitHub Issues: [mark_five_amr repository]
- ROS Answers: Tag with `mark-five-arm`
