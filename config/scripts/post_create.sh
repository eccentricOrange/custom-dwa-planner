#!/bin/bash

# source global ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

# install dependencies
cd $WORKSPACE && \
sudo apt update -y && \
rosdep update

# install dependencies
sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO

# prebuild workspace
colcon build --symlink-install

# source script for convenience functions
echo "source $WORKSPACE/src/config/scripts/convenience_functions.sh" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc