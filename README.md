# Mark Five AMR

Jetson Nano based mobile robot with SLAM capability, powered by **ROS2 Jazzy**.

## Overview

Mark Five AMR is a differential-drive robot platform designed for autonomous navigation, object recognition, and manipulation tasks. The robot uses:
- **Jetson Nano** as the main computer
- **Arduino Mega** for motor control and encoder reading
- **Sabertooth 2x12** motor driver (recommended) or L293DNE H-Bridge
- **Intel RealSense D435** depth camera for perception

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

The Arduino firmware uses **ros2arduino** for ROS2 communication.

### Install ros2arduino Library
1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries
3. Search "ros2arduino" → Install

### Upload Firmware
```bash
# For Sabertooth 2x12 motor driver (recommended)
arduino_ws/Robot_Node_Sabertooth/Robot_Node_Sabertooth.ino

# For L293DNE H-Bridge
arduino_ws/Robot_Node/Robot_Node.ino
```

### Run micro-ROS Agent
The Arduino communicates with ROS2 via a micro-ROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `bringup.launch.py` | Full robot bringup (serial + odometry + teleop) |
| `robot.launch.py` | Jetson-only nodes (for distributed mode) |
| `workstation.launch.py` | Workstation nodes (teleop + RViz) |
| `odometry.launch.py` | Odometry node only |
| `camera.launch.py` | RealSense D435 camera |
| `teleop_keyboard.launch.py` | Keyboard teleoperation |
| `teleop_joy.launch.py` | Joystick teleoperation |

### Examples

```bash
# Full robot bringup
ros2 launch mark_five_bot bringup.launch.py

# Distributed mode - on Jetson
ros2 launch mark_five_bot robot.launch.py camera:=true

# Distributed mode - on Workstation
ros2 launch mark_five_bot workstation.launch.py teleop:=joy
```

## Configuration

All parameters are stored in YAML config files:

```
src/mark_five_bot/config/
├── robot_params.yaml    # Physical robot parameters
├── odometry.yaml        # Odometry node settings
├── camera.yaml          # RealSense camera settings
└── teleop.yaml          # Teleop (joystick/keyboard) settings
```

## Teleoperation

### Keyboard Control
```bash
ros2 launch mark_five_bot teleop_keyboard.launch.py
```

### Joystick Control
```bash
ros2 launch mark_five_bot teleop_joy.launch.py
```
Move the left joystick to control the robot. Hold the enable button (Button 2) while moving.

### Manual cmd_vel Publishing
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

## Odometry

The odometry node computes robot position from encoder ticks using differential drive kinematics.

### Verify Odometry
```bash
# Check odometry topic
ros2 topic echo /odom

# Check TF
ros2 run tf2_ros tf2_echo odom base_footprint

# View TF tree
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
| Max Motor Speed | 130 RPM |
| Velocity Range | 0.187 - 0.374 m/s |

## Project Structure

```
mark_five_amr/
├── src/
│   ├── mark_five_bot/           # Main robot package
│   │   ├── config/              # YAML configuration files
│   │   ├── launch/              # Python launch files
│   │   └── src/                 # C++ nodes
│   └── mark_five_description/   # URDF and visualization
├── arduino_ws/                  # Arduino firmware
├── docker/                      # Docker configuration
└── docs/                        # Documentation
```

## Documentation

- [Project Roadmap](docs/01-Roadmap.md)
- [System Architecture](docs/02-Architecture.md)
- [Package Reference](docs/05-Package-Reference.md)
- [CLAUDE.md](CLAUDE.md) - Technical reference for AI assistants

## References

- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- ros2arduino: https://github.com/ROBOTIS-GIT/ros2arduino
- Differential Drive Math: http://wiki.ros.org/diff_drive_controller
- Project Inspiration: https://github.com/danielsnider/ros-rover
