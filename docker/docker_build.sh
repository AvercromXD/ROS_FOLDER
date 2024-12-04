#!/usr/bin/bash

# Set Contianer Name
CONTAINER_NAME="ros2_container"

echo "Building ROS2-Humble Container"
docker build --rm -t $CONTAINER_NAME:latest --build-arg HOST_GID=$(id -g) . 

echo "Docker Build Completed"
