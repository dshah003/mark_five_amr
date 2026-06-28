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

# Create directory for persistent Docker data (bash history, etc.)
DOCKER_DATA_DIR="${MARKY_ROOT}/docker/.docker_data"
mkdir -p ${DOCKER_DATA_DIR}

# Create bash_history file if it doesn't exist
BASH_HISTORY_FILE="${DOCKER_DATA_DIR}/bash_history"
if [ ! -f "${BASH_HISTORY_FILE}" ]; then
    touch "${BASH_HISTORY_FILE}"
    echo "Created bash history file at ${BASH_HISTORY_FILE}"
fi

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
    -v ${BASH_HISTORY_FILE}:/root/.bash_history:rw \
    -v $HOME/.ssh:/root/.ssh:ro \
    --env="QT_X11_NO_MITSHM=1" \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --env="XAUTHORITY=/root/.Xauthority" \
    --privileged \
    --network=host \
    --ipc=host \
    --env=ROS_DOMAIN_ID=5 \
    --env=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    --env=ROS_STATIC_PEERS="192.168.1.176;192.168.1.169" \
    --env=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
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

# ROS2 Distributed networking
export ROS_DOMAIN_ID=5
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="192.168.1.176;192.168.1.169"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
