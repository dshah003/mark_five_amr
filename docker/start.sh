#!/usr/bin/env bash

DOCKER_NAME=mark_five_robot

# Allow Docker containers to access X11 display
xhost +local:docker > /dev/null 2>&1

echo "Stopping existing containers if any..."
docker stop ${DOCKER_NAME} &> /dev/null
docker rm ${DOCKER_NAME} &> /dev/null

# Edit this as per your dir structure.
MARKY_ROOT=$(dirname "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)")
echo "Setting Marky Root as ${MARKY_ROOT}"

echo "Starting docker container"

# Build video device arguments for RealSense camera
VIDEO_DEVICES=""
for dev in /dev/video*; do
    if [ -e "$dev" ]; then
        VIDEO_DEVICES="${VIDEO_DEVICES} --device ${dev}"
    fi
done

# Check for Arduino device
ARDUINO_DEV=""
if [ -e "/dev/ttyACM0" ]; then
    ARDUINO_DEV="--device /dev/ttyACM0"
fi

# Check for joystick device
JOYSTICK_DEV=""
if [ -e "/dev/input/js0" ]; then
    JOYSTICK_DEV="--device /dev/input/js0"
fi

# Parse command line arguments
DISTRIBUTED_MODE=false
for arg in "$@"; do
    case $arg in
        --distributed)
            DISTRIBUTED_MODE=true
            shift
            ;;
    esac
done

docker run \
    -d \
    -v ${MARKY_ROOT}:/root/mark_five_amr:rw \
    --env="QT_X11_NO_MITSHM=1" \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --env="XAUTHORITY=/root/.Xauthority" \
    --privileged \
    --network=host \
    --env=ROS_DOMAIN_ID=0 \
    ${ARDUINO_DEV} \
    ${JOYSTICK_DEV} \
    -v /dev/bus/usb:/dev/bus/usb \
    ${VIDEO_DEVICES} \
    -h ${DOCKER_NAME} \
    --name ${DOCKER_NAME} mark_five:0.1 sleep infinity

docker exec -u root ${DOCKER_NAME} sh -c "echo 127.0.0.1 ${DOCKER_NAME} >> /etc/hosts"

# Configure bashrc (needed because mount overwrites container's /root)
docker exec ${DOCKER_NAME} sh -c 'cat >> ~/.bashrc << "EOF"

# ROS2 Jazzy setup
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f ~/mark_five_amr/install/setup.bash ]; then
    source ~/mark_five_amr/install/setup.bash
fi

force_color_prompt=yes
EOF
'

echo ""
echo "Container started. Use './bash.sh' to access the shell."
echo ""
echo "Quick start commands:"
echo "  colcon build"
echo "  source install/setup.bash"
echo "  ros2 launch mark_five_bot bringup.launch.py"
