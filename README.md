# Mark Five AMR

Jetson Nano based mobile robot with SLAM and autonomous navigation capability, powered by **ROS2 Jazzy + Nav2**.

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

The Arduino uses a lightweight serial protocol. A Python ROS2 node (`serial_bridge.py`) handles the translation between serial and ROS2 topics.

### Upload Firmware
```bash
# For Sabertooth 2x12 motor driver (recommended)
arduino_ws/Robot_Node_Sabertooth/Robot_Node_Sabertooth.ino

# For L293DNE H-Bridge
arduino_ws/Robot_Node/Robot_Node.ino
```

### Install Sabertooth Library
1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries
3. Search "Sabertooth" → Install (by Dimension Engineering)

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `bringup.launch.py` | Full robot bringup (serial + odometry + teleop + optional nav) |
| `robot.launch.py` | Jetson-only nodes (for distributed mode) |
| `workstation.launch.py` | Workstation nodes (teleop + RViz) |
| `navigation.launch.py` | Nav2 navigation stack (SLAM or localization) |
| `slam.launch.py` | SLAM mapping with slam_toolbox |
| `localization.launch.py` | Localization with pre-built map (AMCL) |
| `odometry.launch.py` | Odometry node only |
| `camera.launch.py` | RealSense D435 camera |
| `teleop_keyboard.launch.py` | Keyboard teleoperation |
| `teleop_joy.launch.py` | Joystick teleoperation |

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
ros2 launch mark_five_bot robot.launch.py camera:=true

# Distributed mode - on Workstation
ros2 launch mark_five_bot workstation.launch.py teleop:=joy
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

## Configuration

All parameters are stored in YAML config files:

```
src/mark_five_bot/config/
├── robot_params.yaml        # Physical robot parameters
├── odometry.yaml            # Odometry node settings
├── camera.yaml              # RealSense camera settings
├── teleop.yaml              # Teleop (joystick/keyboard) settings
├── nav2_params.yaml         # Nav2 navigation parameters
└── slam_toolbox_params.yaml # SLAM mapping parameters
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
│   │   ├── maps/                # Saved map files
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
- Nav2 Navigation: https://nav2.org/
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- Differential Drive Math: http://wiki.ros.org/diff_drive_controller
- Project Inspiration: https://github.com/danielsnider/ros-rover
