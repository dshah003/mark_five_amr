#!/usr/bin/env bash

echo "Stopping exisitng containers if any. . ."
docker stop mark_five &> /dev/null
docker rm mark_five &> /dev/null

# Edit this as per your dir structure.
MARKY_ROOT="${HOME}/mark_five_amr/"
echo "Setting Marky Root as ${MARKY_ROOT}"

echo "Starting docker container"

docker run \
    -d \
    -v ${MARKY_ROOT}:${HOME}:rw \
    --privileged \
    --network=host \
    -h mark_five_robot \
    --name mark_five mark_five:0.1 sleep infinity