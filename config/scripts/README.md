# Scripts for handling the development environment

The Docker environment used in this repository is build from the [Dockerfile in the Devcontainer configuration](/.devcontainer/Dockerfile).

The scripts should be run from the root of the repository.

## Quickstart guide for running Docker containers manually

1.  Pull the base image and build the container.

    ```bash
    ./config/scripts/devel.sh
    ```

2.  Enter the container.

    ```bash
    ./config/scripts/enter_bash.sh
    ```

3.  When you are done, stop the container.

    ```bash
    ./config/scripts/stop.sh
    ```

## Overview
- You may either run the Docker containers directly from a command line, or use them as [Dev Containers](https://containers.dev/).
- Usage steps are very similar for both systems.

## Functions of each script

| Script  | Description | Used by Devcontainer |
| --- | --- | --- |
| [devel.sh](devel.sh) | Uses the Dockerfile to create and start a Docker container | No |
| [enter_bash.sh](enter_bash.sh) | Runs the image created by `devel.sh` | No |
| [stop.sh](stop.sh) | Stop and delete the container created by `devel.sh` | No |
| [convenience_functions.sh](convenience_functions.sh) | Create bash aliases for building and sourcing the ROS2 environment. Automatically sourced by `post_create.sh`. | Yes |
| [post_create.sh](post_create.sh) | Configure the workspace and source `convenience_functions.sh` once the container run for the first time. Automatically called by `devel.sh`. | Yes |


## Convenience functions
A couple of convenience functions are provided, implemented as aliases in `~/.bashrc`, if you use the post-create script to set up your environment.

-   **`wss` Source Workspace**

    This function sources the workspace setup file.

    It is equivalent to running the following command:

    ```bash
    source $WORKSPACE/install/setup.bash && echo "Sourced workspace"
    ```

-   **`wsb` Build Workspace**

    This function builds the workspace and then sources it.

    It is equivalent to running the following command:

    ```bash
    cd $WORKSPACE && colcon build --symlink-install && wss
    ```

-   **`rdi` ROS2 Dependency Install**

    This function installs all ROS2 dependencies for the workspace using `rosdep`.

    It is equivalent to running the following command:

    ```bash
    sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO
    ```