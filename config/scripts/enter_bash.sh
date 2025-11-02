#!/bin/bash

CONTAINER_WORKSPACE=/home/ros/dwa_ws
CONTAINER_NAME="dwa_container"
IMAGE_NAME="dwa_image"

xhost +local:root
docker exec -it \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
--workdir="$WORKSPACE" \
$CONTAINER_NAME bash
