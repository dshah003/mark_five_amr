#!/usr/bin/env bash
#
# Mark Five AMR - Docker Container Start Script
#
# Usage:
#   ./start.sh              # Standalone mode (starts local roscore)
#   ./start.sh --distributed # Distributed mode (connects to workstation roscore)
#

DOCKER_NAME=mark_five_robot
DISTRIBUTED_MODE=false

# Parse arguments
if [ "$1" == "--distributed" ] || [ "$1" == "-d" ]; then
    DISTRIBUTED_MODE=true
    echo "Starting in DISTRIBUTED mode (no local roscore)"
else
    echo "Starting in STANDALONE mode (local roscore)"
fi

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

docker run \
    -d \
    -v ${MARKY_ROOT}:${HOME}:rw \
    --env="QT_X11_NO_MITSHM=1" \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    --network=host \
    --device /dev/ttyACM0 \
    --device /dev/input/js0 \
    -v /dev/bus/usb:/dev/bus/usb \
    ${VIDEO_DEVICES} \
    -h ${DOCKER_NAME} \
    --name ${DOCKER_NAME} mark_five:0.1 sleep infinity

docker exec -u root ${DOCKER_NAME} sh -c "echo 127.0.0.1 ${DOCKER_NAME} >> /etc/hosts"
docker exec ${DOCKER_NAME} sh -c "echo 'force_color_prompt=yes' >> ~/.bashrc"

# Start roscore only in standalone mode
if [ "$DISTRIBUTED_MODE" = false ]; then
    echo "Starting local roscore..."
    docker exec -d ${DOCKER_NAME} bash -c ". /opt/ros/melodic/setup.bash && roscore"
    echo "Roscore started. Use './bash.sh' to access container."
else
    echo ""
    echo "=========================================="
    echo "  DISTRIBUTED MODE"
    echo "=========================================="
    echo "  No local roscore started."
    echo ""
    echo "  Inside container, run:"
    echo "    source ~/mark_five_amr/scripts/env_robot.sh"
    echo "    roslaunch mark_five_bot robot.launch"
    echo "=========================================="
fi