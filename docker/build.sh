#!/usr/bin/env bash

echo "Building mark_five docker (ROS2 Jazzy)"
echo "Using HOME=${HOME}"

docker build \
    --build-arg HOME=${HOME} \
    -t mark_five:0.1 .
