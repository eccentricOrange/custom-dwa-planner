#!/bin/bash

# source workspace alias
alias wss="source $WORKSPACE/install/setup.bash && echo \"Sourced workspace\""

# build workspace alias
alias wsb="cd $WORKSPACE && colcon build --symlink-install && wss"

# god forsaken rosdep install alias
## if environment variable PLATFORM is "sbc", skip the "int_brain_gazebo" package
if [ "$PLATFORM" == "sbc" ]; then

    # Read the keys to ignore from the file into a variable
    IGNORED_KEYS=$(cat "$WORKSPACE/src/config/sbc-rosdep-ignores.txt")
    alias rdi="sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=\"$IGNORED_KEYS\""

else
    alias rdi="sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO"
fi

# enable colcon argument completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash