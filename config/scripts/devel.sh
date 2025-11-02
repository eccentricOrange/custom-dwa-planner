#!/bin/bash

CONTAINER_WORKSPACE=/home/ros/dwa_ws
CONTAINER_NAME="dwa_container"
IMAGE_NAME="dwa_image"

config/scripts/stop.sh

# get Git submodules
git submodule update --init --recursive

docker build -t $IMAGE_NAME -f .devcontainer/Dockerfile .

docker run -it -d \
--net=host \
--name $CONTAINER_NAME \
--privileged \
--volume $PWD:$CONTAINER_WORKSPACE/src \
--volume /dev:/dev \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
--env "TERM=xterm-256color" \
--user ros \
$IMAGE_NAME

docker exec -it \
$CONTAINER_NAME \
$CONTAINER_WORKSPACE/src/config/scripts/post_create.sh