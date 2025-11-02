# Custom DWA Planner

## Usage

Prerequisites:
-   Docker installed on your system.
-   Git installed to clone the repository and submodules.
-   Tested with a Linux Ubuntu 24.04 host system, so this is the preferred OS.
-   (Optional) VSCode with Devcontainer extension if you wish to use Devcontainers.

### Container setup
This project is built using a Docker environment, which may be used either with Devcontainers in VSCode or directly with Docker.

Please see the [Setup Scripts README](/config/scripts/README.md) for instructions and shell scripts to help quickly launch the workspace without dependence on Devcontainers.

In order to use with Devcontainers, make sure you have the [Devcontainer extension for VSCode](https://code.visualstudio.com/docs/remote/containers) installed. Then, simply open this folder in VSCode and it should prompt you to reopen in a container.

### Running the example code
The example DWA planner is implemented as a single node, and is provided with a Python launch file. By default, the launch file will run:
-   The Gazebo simulator with a Turtlebot3 Burger robot
-   The custom DWA planner node
-   RViz for visualization with a pre-configured display

If you have set up the container correctly, you can run the example launch file with, it should have already installed all dependencies and built the workspace. In this case, you can run:

```bash
ros2 launch dwa_planner dwa.launch.py
```

### Arguments in the launch file

| Argument Name | Default Value | Description |
| --- | --- | --- |
| `launch_simulator` | `true` | Whether to launch the Gazebo simulator with the Turtlebot3 robot. Set to `false` if you want to use a physical robot instead. |
| `launch_teleop` | `false` | Whether to launch a teleoperation node (in a separate window) to control the robot using keyboard input. |
| `use_sim_time` | `true` | Whether to use simulation time. Should be `true` when using Gazebo, and `false` when using a physical robot. |


## Implementation details

### High-Performance Vectorized Logic
The core DWA evaluation pipeline uses `numpy` vectorization to avoid slow Python for loops. The entire process of trajectory prediction, collision checking, and scoring is performed "all-at-once" on large arrays. This vectorized approach allows it to evaluate thousands of trajectories ($50 \times 50 = 2500$ in the config) in real-time.

### Key DWA Algorithm Steps
1.  **Define Dynamic Window**

    Calculates the range of reachable velocities $(v_r, \omega_r)$ based on the robot's current velocity $(v_c, \omega_c)$, acceleration limits $(a_l, a_a)$, and the time step $\mathrm{d}t$.

    $$v_r \in [\max(v_c - a_l \cdot \mathrm{d}t, v_{min}), \min(v_c + a_l \cdot \mathrm{d}t, v_{max})]$$

    $$\omega_r \in [\max(\omega_c - a_a \cdot \mathrm{d}t, (-\omega_{max})), \min(\omega_c + a_a \cdot \mathrm{d}t, \omega_{max})]$$

2.  **Check for Collisions**

    Calculates the minimum distance from each trajectory to the nearest obstacle. If this distance is less than the $\text{robot radius}$, the trajectory is marked as invalid.
    
    $$\text{min dist} < \text{robot radius}$$

3.  **Score Trajectories**

    Selects the best command by finding the trajectory with the lowest total cost, calculated as a weighted sum of three factors.

    $$\text{Cost} = \alpha \cdot \text{goal cost} + \beta \cdot \text{obstacle cost} + \gamma \cdot \text{velocity cost}$$