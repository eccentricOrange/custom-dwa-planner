#!/bin/bash

# take ownership of folders
# sudo chown -R ubuntu:ubuntu $WORKSPACE

# source global ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

# install dependencies
cd $WORKSPACE && \
sudo apt update -y && \
rosdep update


## if environment variable PLATFORM is "sbc", skip the "int_brain_gazebo" package
if [ "$PLATFORM" == "sbc" ]; then

    # Read the keys to ignore from the file into a variable
    IGNORED_KEYS=$(cat "$WORKSPACE/src/config/sbc-rosdep-ignores.txt")
    sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO --skip-keys="$IGNORED_KEYS"

else
    sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO
fi

colcon build --symlink-install

# source script for convenience functions
echo "source $WORKSPACE/src/config/scripts/convenience_functions.sh" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc