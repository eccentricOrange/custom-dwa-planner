#!/bin/bash

CONTAINER_NAME="dwa_container"
docker stop $CONTAINER_NAME && docker rm $CONTAINER_NAME
xhost -local:root