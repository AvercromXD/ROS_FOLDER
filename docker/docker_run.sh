#!/usr/bin/env bash

BASH_HISTORY_FILE=${PWD%/*}/docker/.bash_history
BASH_RC_FILE=${PWD%/*}/docker/.bashrc

CONTAINER_NAME="ros2_container"
DOCKER_USER="dockerian"

docker_count=$(docker ps -a | grep $CONTAINER_NAME | wc -l)
((docker_count=docker_count+1))

XAUTH=/tmp/.docker.xauth_$docker_count
sleep 0.1
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


docker run -it  --rm\
    --name $CONTAINER_NAME-$docker_count \
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
    --privileged \
    $CONTAINER_NAME

echo "Docker container exited."
