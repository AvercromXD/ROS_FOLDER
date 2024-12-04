#!/usr/bin/env bash

BASH_HISTORY_FILE=${PWD%/*}/docker/.bash_history
BASH_RC_FILE=${PWD%/*}/docker/.bashrc

CONTAINER_NAME="ros2_vdb"
DOCKER_USER="dockerian"

XAUTH=/tmp/.docker.xauth_ros2_vdb_cont
sleep 0.1
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it  --rm\
    --name $CONTAINER_NAME \
    --volume="${PWD%/*}:/home/$DOCKER_USER" \
    --volume="$BASH_HISTORY_FILE:/home/$DOCKER_USER/.bash_history" \
    --volume="$BASH_RC_FILE:/home/$DOCKER_USER/.bashrc" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --env="XAUTHORITY=$XAUTH" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --workdir="/home/$DOCKER_USER" \
    --net=host \
    --ipc=host \
    --pid=host\
    --privileged \
    avercromxd/ros2_vdb:latest

echo "Docker container exited."
