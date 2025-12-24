#!/usr/bin/env bash

DOCKER_NAME=mark_five_robot
echo "Stopping exisitng containers if any. . ."
docker stop ${DOCKER_NAME} &> /dev/null
docker rm ${DOCKER_NAME} &> /dev/null

# Edit this as per your dir structure.
MARKY_ROOT=$(dirname "$(pwd)")
echo "Setting Marky Root as ${MARKY_ROOT}"

echo "Starting docker container"

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
    -h ${DOCKER_NAME} \
    --name ${DOCKER_NAME} mark_five:0.1 sleep infinity

docker exec -u root ${DOCKER_NAME} sh -c "echo 127.0.0.1 ${DOCKER_NAME} >> /etc/hosts"

# docker exec -u root ${DOCKER_NAME} sh -c "source /opt/ros/melodic/setup.bash && roscore &> /dev/null &"
docker exec -d mark_five_robot bash -c ". /opt/ros/melodic/setup.bash && roscore"