ARG ROS_DISTRO=humble

## Get base image
FROM ros:$ROS_DISTRO-ros-base

## Config args
ARG USERNAME=ros
ARG WORKSPACE=/home/$USERNAME/dwa_ws
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd -g $USER_GID $USERNAME \
    && useradd -u $USER_UID -g $USER_GID -m -d /home/$USERNAME -s /bin/bash $USERNAME

## Make $USERNAME a sudo user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

## Update system
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get update -y

## Install basic tools
RUN apt-get install --no-install-recommends -y \
    vim \
    bat \
    python3-pip \
    python3-venv \
    net-tools \
    tree \
    bash-completion \
    wget \
    xterm \
    git \
    python3-vcstool \
    python3-colcon-common-extensions

## Add the GitHub CLI repository
RUN mkdir -p -m 755 /etc/apt/keyrings \
    && out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    && cat $out | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
    && chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
    && mkdir -p -m 755 /etc/apt/sources.list.d \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null

## Install GitHub CLI
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get update -y \
    && apt-get install gh --no-install-recommends -y

## Install ROS packages
# Install Xacro
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-xacro

# Install ros2 control
# RUN apt-get install --no-install-recommends -y \
#     ros-$ROS_DISTRO-ros2-control \
#     ros-$ROS_DISTRO-ros2-controllers \
#     ros-$ROS_DISTRO-rqt-controller-manager

# Install sensor fusion
# RUN apt-get install --no-install-recommends -y \
#     ros-$ROS_DISTRO-robot-localization

# Install RPLidar SDK
# RUN apt-get install --no-install-recommends -y \
#     ros-$ROS_DISTRO-rplidar-ros

## Set default shell to bash 
SHELL ["/bin/bash", "-c"]

## Add to groups
RUN usermod -aG dialout $USERNAME
RUN usermod -aG video $USERNAME

## Set username
USER $USERNAME
RUN rosdep update


## Set variables
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_DOMAIN_ID=42
ENV WORKSPACE=$WORKSPACE

## Create workspace
RUN mkdir -p $WORKSPACE

# Convenience scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f $WORKSPACE/install/setup.bash ]; then source $WORKSPACE/install/setup.bash && echo \"Sourced workspace\"; fi" >> /home/$USERNAME/.bashrc

ENV PLATFORM=host

USER root

## Update system
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get update -y

# Install Gazebo Classic (Gazebo11)
RUN apt-get install --no-install-recommends -y \
    gazebo \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-ros

# Install teleop packages
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-joy

# Install Utils
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-plotjuggler-ros

USER $USERNAME
WORKDIR $WORKSPACE

LABEL org.opencontainers.image.authors="eccentricOrange"