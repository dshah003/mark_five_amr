# Mark Five AMR

Jetson Nano based mobile robot with SLAM and autonomous navigation capability, powered by **ROS2 Jazzy + Nav2**.

## Overview

Mark Five AMR is a differential-drive robot platform designed for autonomous navigation and manipulation tasks. The robot uses:
- **Jetson Nano** as the main computer
- **Arduino Mega** for motor control and encoder reading
- **2x BTS7960 (IBT-2)** motor driver modules (one per motor)
- **Intel RealSense D435** depth camera for perception
- **ICM-20948 9-DOF IMU** for sensor fusion (gyro-assisted odometry)
- **Lite Arm i2** 3-DOF robotic arm with PCA9685 PWM driver

## Quick Start

### Prerequisites

#### Install Docker
Follow instructions at https://docs.docker.com/engine/install/ubuntu/

#### Post Installation
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker  # Or logout and log back in
```

### External Dependencies (Build from Source)

The following packages must be cloned into `src/` before building:

```bash
cd ~/mark_five_amr/src

# ICM-20948 IMU driver (requires: pip3 install sparkfun-qwiic-icm20948)
# Note: after cloning, icm20948_node.py must be patched to remove the
# imu.connected check, which fails on Jetson Nano's I2C implementation.
# See the IMU Troubleshooting section below.
git clone https://github.com/norlab-ulaval/ros2_icm20948.git
```

### Docker Setup

```bash
# Build Docker image
cd docker && ./build.sh

# Start container
./start.sh

# Access shell
./bash.sh
```

### Build and Run

```bash
# Inside Docker container
cd ~/mark_five_amr
colcon build
source install/setup.bash

# Launch full robot bringup
ros2 launch mark_five_bot bringup.launch.py

# Or with camera
ros2 launch mark_five_bot bringup.launch.py use_camera:=true
```

## Arduino Setup

The Arduino uses a lightweight serial protocol. A Python ROS2 node (`serial_bridge.py`) handles the translation between serial and ROS2 topics.

### Upload Firmware
```bash
# For BTS7960 motor drivers (current hardware, no library needed)
arduino_ws/Robot_Node_BTS7960/Robot_Node_BTS7960.ino

# For L293DNE H-Bridge (legacy)
arduino_ws/Robot_Node/Robot_Node.ino
```

## Launch Files

### Standalone Mode (single machine)

| Launch File | Description |
|-------------|-------------|
| `bringup.launch.py` | Full robot bringup (serial + odometry + IMU/EKF + optional nav) |

### Distributed Mode (Jetson + Workstation)

| Launch File | Description |
|-------------|-------------|
| `robot.launch.py` | Jetson-side nodes (serial, odometry, IMU, EKF, camera) |
| `workstation.launch.py` | Workstation-side nodes (SLAM/Nav2, RViz, optional teleop) |

### Component Launch Files

| Launch File | Description |
|-------------|-------------|
| `serial.launch.py` | Serial bridge to Arduino |
| `odometry.launch.py` | Encoder-based odometry node |
| `imu.launch.py` | ICM-20948 IMU driver |
| `ekf.launch.py` | EKF sensor fusion (odometry + IMU) |
| `camera.launch.py` | RealSense D435 camera |
| `teleop_keyboard.launch.py` | Keyboard teleoperation |
| `teleop_joy.launch.py` | Joystick teleoperation |
| `navigation.launch.py` | Nav2 navigation stack (SLAM or localization) |
| `slam.launch.py` | Laser SLAM with slam_toolbox |
| `rtabmap_slam.launch.py` | Visual SLAM with RTAB-Map |
| `localization.launch.py` | Localization with pre-built map (AMCL) |
| `mission_manager.launch.py` | Waypoint mission manager |

### Arm Launch Files

| Launch File | Description |
|-------------|-------------|
| `arm.launch.py` | Arm controller only (programmatic use) |
| `arm_test.launch.py` | Arm controller + GUI sliders + RViz |
| `display.launch.py` | URDF visualization only (no hardware) |

### Examples

```bash
# Full robot bringup
ros2 launch mark_five_bot bringup.launch.py

# With camera
ros2 launch mark_five_bot bringup.launch.py use_camera:=true

# SLAM mode (build maps)
ros2 launch mark_five_bot bringup.launch.py use_camera:=true use_nav:=true nav_mode:=slam

# Navigation mode (use saved map)
ros2 launch mark_five_bot bringup.launch.py use_camera:=true use_nav:=true nav_mode:=localization map:=/path/to/map.yaml

# Distributed mode - on Jetson
ros2 launch mark_five_bot robot.launch.py
ros2 launch mark_five_bot robot.launch.py camera_mode:=visual_slam

# Distributed mode - on Workstation
ros2 launch mark_five_bot workstation.launch.py slam_mode:=visual teleop:=keyboard
```

## Navigation

### SLAM Mode (Building Maps)
```bash
# Launch robot with SLAM
ros2 launch mark_five_bot bringup.launch.py use_camera:=true use_nav:=true nav_mode:=slam

# Drive around to build the map, then save it
ros2 run nav2_map_server map_saver_cli -f ~/mark_five_amr/src/mark_five_bot/maps/my_map
```

### Localization Mode (Using Saved Maps)
```bash
# Launch with pre-built map
ros2 launch mark_five_bot bringup.launch.py use_camera:=true use_nav:=true nav_mode:=localization map:=/full/path/to/map.yaml

# Open RViz with navigation config
rviz2 -d ~/mark_five_amr/src/mark_five_description/rviz/navigation.rviz

# Set initial pose (2D Pose Estimate tool in RViz)
# Set goal (2D Goal Pose tool in RViz)
```

### Navigation Parameters
Key parameters in `src/mark_five_bot/config/nav2_params.yaml`:
- **Robot radius:** 0.12m
- **Max velocity:** 0.20 m/s linear, 1.0 rad/s angular
- **Controller:** Regulated Pure Pursuit
- **Planner:** NavFn

### Waypoint Mission Manager
```bash
# Launch navigation with mission manager
ros2 launch mark_five_bot navigation.launch.py mode:=localization use_mission_manager:=true

# Load waypoints
ros2 topic pub --once /mission/waypoints geometry_msgs/PoseArray \
  "{header: {frame_id: 'map'}, poses: [\
    {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}},\
    {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}\
  ]}"

# Start mission
ros2 service call /mission/start std_srvs/srv/Trigger
```

## IMU and Sensor Fusion

The ICM-20948 9-DOF IMU is fused with wheel odometry using an EKF (`robot_localization` package). Only gyroscope data is used (accelerometer disabled due to vibration noise), providing ~40% reduction in yaw drift during turns.

**Hardware connection (Jetson Nano I2C):**
```
ICM-20948 VIN → Pin 1  (3.3V)
ICM-20948 GND → Pin 6  (GND)
ICM-20948 SDA → Pin 3  (I2C Bus 1 SDA)
ICM-20948 SCL → Pin 5  (I2C Bus 1 SCL)
```

**Verify I2C connection:**
```bash
sudo i2cdetect -y -r 1  # Should show 0x68
```

**Disable IMU if not connected:**
```bash
ros2 launch mark_five_bot robot.launch.py imu:=false
```

### IMU Troubleshooting

**`isDeviceConnected` returns False on Jetson Nano:**
The SparkFun qwiic library's `isDeviceConnected()` uses an I2C probe method that doesn't work on Jetson Nano, even when the device is present and fully functional. The node crashes on `begin()` as a result.

Fix — remove the `connected` check in `src/src/ros2_icm20948/ros2_icm20948/icm20948_node.py`:
```python
# Remove these lines:
if not self.imu.connected:
    self.logger.info("The Qwiic ICM20948 device isn't connected...")

# Keep:
self.imu.begin()
```

Then rebuild:
```bash
colcon build --packages-select ros2_icm20948
```

## Robotic Arm (Lite Arm i2)

3-DOF parallel linkage arm (Thingiverse 480446) with 3x Power HD 1501 MG servos, controlled via PCA9685 PWM driver.

**Hardware connection (shares I2C bus with IMU):**
```
PCA9685 VCC → Pin 1  (3.3V)
PCA9685 GND → Pin 6  (GND)
PCA9685 SDA → Pin 3  (I2C Bus 1 SDA)
PCA9685 SCL → Pin 5  (I2C Bus 1 SCL)
PCA9685 V+  → External 5-6V supply (required for servo power)
```

**Quick start:**
```bash
# On Jetson: run arm controller
ros2 launch mark_five_arm arm_test.launch.py use_rviz:=false use_gui:=false

# Move arm
ros2 topic pub --once /arm/joint_commands sensor_msgs/JointState \
  "{name: ['base', 'shoulder', 'elbow'], position: [0.0, 0.5, -0.3]}"

# Home / relax
ros2 service call /arm/home std_srvs/srv/Trigger
ros2 service call /arm/relax std_srvs/srv/Trigger
```

## Configuration

All parameters are stored in YAML config files:

```
src/mark_five_bot/config/
├── robot_params.yaml          # Physical robot parameters
├── odometry.yaml              # Odometry node settings
├── robot_localization.yaml    # EKF sensor fusion (odometry + IMU)
├── camera.yaml                # RealSense camera settings
├── teleop.yaml                # Teleop (joystick/keyboard) settings
├── nav2_params.yaml           # Nav2 navigation parameters
├── slam_toolbox_params.yaml   # Laser SLAM parameters
├── rtabmap_params.yaml        # Visual SLAM parameters
└── mission_manager.yaml       # Waypoint mission manager settings

src/mark_five_arm/config/
└── arm_params.yaml            # Servo PWM calibration and joint limits
```

## Teleoperation

### Keyboard Control
```bash
# Must run directly (not via launch file)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Joystick Control
```bash
ros2 launch mark_five_bot teleop_joy.launch.py
```
Hold the enable button (Button 2) while moving the left joystick.

### Manual cmd_vel Publishing
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

## Odometry

The odometry node computes robot position from encoder ticks using differential drive kinematics. When the IMU is connected, the EKF fuses odometry with gyroscope data for improved yaw accuracy.

### Verify Odometry
```bash
ros2 topic echo /odom                              # Raw wheel odometry
ros2 topic echo /odometry/filtered                 # EKF-fused odometry
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
```

## URDF Visualization

```bash
ros2 launch mark_five_description display.launch.py
```

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 0.055 m |
| Wheel Base | 0.14 m |
| Encoder Ticks per Revolution | 540 |
| Ticks per Meter | 3125 |
| Max Velocity | ~0.37 m/s |

## Project Structure

```
mark_five_amr/
├── src/
│   ├── mark_five_bot/           # Main robot package
│   │   ├── config/              # YAML configuration files
│   │   ├── launch/              # Python launch files
│   │   ├── maps/                # Saved map files
│   │   ├── mark_five_bot/       # Python nodes (serial_bridge, mission_manager)
│   │   └── src/                 # C++ nodes (odometry_node)
│   ├── mark_five_description/   # Robot URDF and RViz configs
│   ├── mark_five_arm/           # Robotic arm package (Lite Arm i2)
│   └── ros2_icm20948/           # IMU driver (cloned from norlab-ulaval)
├── arduino_ws/                  # Arduino firmware
├── docker/                      # Docker configuration
└── docs/                        # Documentation
```

## Documentation

- [Project Roadmap](docs/01-Roadmap.md)
- [System Architecture](docs/02-Architecture.md)
- [Package Reference](docs/05-Package-Reference.md)

## References

- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- Nav2 Navigation: https://nav2.org/
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- ros2_icm20948: https://github.com/norlab-ulaval/ros2_icm20948
- Lite Arm i2: https://www.thingiverse.com/thing:480446
