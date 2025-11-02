from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # --- Get Package Directories ---
    pkg_dwa_planner = get_package_share_directory('dwa_planner')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_teleop = get_package_share_directory('turtlebot3_teleop')

    # --- Paths ---
    dwa_params_file = Path(pkg_dwa_planner) / 'config' / 'dwa_params.yaml'
    rviz_config_file = Path(pkg_dwa_planner) / 'rviz' / 'dwa_debug.rviz'
    sim_launch_file = Path(pkg_turtlebot3_gazebo) / 'launch' / 'turtlebot3_world.launch.py'
    
    # --- Launch Arguments ---
    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim',
        default_value='true',
        description='Whether to launch the Gazebo simulation',
    )
    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='false',
        description='Whether to launch the keyboard teleoperation node',
    )
    
    # --- Environment Variables ---
    # Set the TurtleBot3 model for all nodes
    set_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger'
    )

    # --- Node Definitions ---

    # 1. DWA Planner Node (Your "Brain")
    dwa_planner_node = Node(
        package='dwa_planner',
        executable='dwa_planner_node',
        name='dwa_planner_node',
        output='screen',
        parameters=[str(dwa_params_file)],
    )

    # 2. RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config_file)],
        output='screen',
    )

    # 3. Gazebo Simulation (Conditional)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch_file)),
        condition=IfCondition(LaunchConfiguration('launch_sim')),
    )

    # 4. Keyboard Teleop (Conditional)
    
    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',  # Launch in a new terminal window
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    return LaunchDescription(
        [
            # Launch Arguments
            launch_sim_arg,
            launch_teleop_arg,
            
            # Environment
            set_model_env,
            
            # Nodes & Other Launch Files
            gazebo_sim,
            dwa_planner_node,
            teleop_node,
            rviz_node,
        ]
    )

