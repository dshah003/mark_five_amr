# mark_five_amr
Jetson nano based mobile robot with SLAM capability

## Setup
The Mark Five AMR runs on Jetson Nano as it's main computer. An Arduino Mega is used to  control motors via motor driver and read encoder data. 
The robot environment is setup on a docker container based on ROS Melodic. 

### Pre-requisites

#### Install docker 

Follow instructions at https://docs.docker.com/engine/install/ubuntu/ to install docker on your host OS.

#### Post Installation

```sh 
sudo groupadd docker 
sudo usermod -aG docker $USER
## After running this, logout and log back in for changes to take effect. OR run the following command.
newgrp docker
```

## Docker container

To run the docker, run the `build.sh` script in `docker/` dir. 
To start the docker run the `start.sh` script once, and then run `bash.sh` script in new terminal everytime to open bash within the docker. 

## Arduino Node
The arduino_ws/Robot_Node/Robot_Node.ino code runs on the arduino Mega. 
The code is referenced from https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/ and tweaked specific to my robot.

### To Run ROS Serial  

```sh
catkin_make
source devel/setup.bash
# Assuming you have roscore running on another terminal.
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200  

# OR run using roslaunch
roslaunch mark_five_bot launch_rosserial.launch
``` 

### For Running the robot via tele-op 

```sh
roslaunch mark_five_bot launch_rosserial_keyboard_teleop.launch 
```

### To Publish cmd_vel from terminal
```sh
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
angular:
  z: -0.1" --once
```

### Run Teleop + Joystick node.  
This publishes to /cmd_vel based on joystick movements. Move the left joystick axis to move the robot. The robot will move while the enable button is pressed. (The Button marked "2" on the controller")
```sh
roslaunch mark_five_bot launch_joystick_teleop.launch  
```


## Notes 

- One Revolution = 540 Encoder ticks. 
- Wheel diameter = 0.055 m
- Wheel Radius = 0.0275 m
- Max Motor speed = 130rpm
- Circumference = 2*pi*r = 0.17279

*If PWM range is 50-100, robot velocity will be in the range:*
- Linear velocity = Angular_Velocity * r OR rpm * Circumference (divide by 60 to get m/s)
- Linear velocity Min = 130 rotations/min * 0.50 duty cycle * 0.17279 m / 60 sec = 0.187 m/s
- Linear velocity Max = 130 rotations/min * 1.0 duty cycle * 0.17279 m / 60 sec = 0.3744 m/s
- Velocity Range = 0.187 - 0.3744 m/s

@TODO: Calculate robot velocity via Encoder and verify the above calculations.

### References:

Math: http://wiki.ros.org/diff_drive_controller
Overall autonomous system design: https://github.com/danielsnider/ros-rover  

