#!/usr/bin/env bash

DOCKER_NAME=mark_five_robot
echo "Stopping exisitng containers if any. . ."
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

# docker exec -u root ${DOCKER_NAME} sh -c "source /opt/ros/melodic/setup.bash && roscore &> /dev/null &"
docker exec -d mark_five_robot bash -c ". /opt/ros/melodic/setup.bash && roscore"