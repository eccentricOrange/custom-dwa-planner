import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import tf_transformations
from tf2_py import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import hypot, atan2, cos, sin, pi
from builtin_interfaces.msg import Time

class DwaPlanner(Node):
    """
    A DWA local planner node for a differential drive robot (TurtleBot3).
    """

    def __init__(self):
        super().__init__('dwa_planner_node')

        self.declare_parameters_and_load()

        # State Variables
        self.current_pose: list[float] = None      # [x, y, yaw]
        self.current_velocity: list[float] = None  # [v, w]
        self.current_scan: LaserScan = None
        self.current_goal: list[float] = None      # [x, y]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS profile for visualization markers (Transient Local)
        marker_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/dwa_trajectories', 
            marker_qos_profile
        )

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        # DWA "Engine" Timer
        self.timer_period = 1.0 / self.controller_frequency
        self.create_timer(self.timer_period, self.run_dwa_logic)

        self.get_logger().info("DWA Planner Node initialized.")
        self.get_logger().info(
            f"Waiting for topics (odom, scan, goal_pose)..."
        )
    
    def declare_parameters_and_load(self):
        """
        Declare and load all DWA parameters from the YAML file.
        """
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('robot_radius', 0.17)
        self.declare_parameter('predict_time', 2.0)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('linear_samples', 11)
        self.declare_parameter('angular_samples', 21)
        self.declare_parameter('controller_frequency', 10.0)
        self.declare_parameter('goal_cost_weight', 1.0)
        self.declare_parameter('obstacle_cost_weight', 2.0)
        self.declare_parameter('velocity_cost_weight', 0.1)

        # Load parameters into class variables
        self.max_v = self.get_parameter('max_linear_speed').value
        self.min_v = self.get_parameter('min_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        self.robot_radius = self.get_parameter('robot_radius').value

        self.predict_time = self.get_parameter('predict_time').value
        self.dt = self.get_parameter('time_step').value
        self.predict_steps = int(self.predict_time / self.dt)

        self.v_samples = self.get_parameter('linear_samples').value
        self.w_samples = self.get_parameter('angular_samples').value
        self.controller_frequency = self.get_parameter('controller_frequency').value

        self.alpha = self.get_parameter('goal_cost_weight').value
        self.beta = self.get_parameter('obstacle_cost_weight').value
        self.gamma = self.get_parameter('velocity_cost_weight').value

    # Callbacks (Store Sensor Data)

    def odom_callback(self, msg: Odometry):
        """Stores the latest odometry and velocity."""
        self.current_velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        pose = msg.pose.pose
        yaw = self.get_yaw_from_quaternion(pose.orientation)
        self.current_pose = [pose.position.x, pose.position.y, yaw]

    def scan_callback(self, msg: LaserScan):
        """Stores the latest laser scan."""
        self.current_scan = msg

    def goal_callback(self, msg: PoseStamped):
        """Stores the latest goal pose."""
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New goal received: {self.current_goal}")

    # The Main "Engine" Loop

    def run_dwa_logic(self):
        """
        The main DWA logic loop, run on a timer.
        """
        # Wait until all required data is available
        if (
            self.current_pose is None
            or self.current_scan is None
            or self.current_goal is None
            or self.current_velocity is None
        ):
            if self.get_clock().now().nanoseconds % 1e9 < 1e8:
                self.get_logger().warn(
                    "Waiting for topics (odom, scan, goal_pose)..."
                )
            return
        
        # Stop planning if goal is reached
        if self.is_goal_reached():
            self.get_logger().info("Goal Reached!")
            self.cmd_vel_pub.publish(Twist())
            self.current_goal = None
            self.publish_trajectories([], []) 
            return

        try:
            # Step 1: Calculate the dynamic window
            v_range, w_range = self.calculate_dynamic_window()

            # Step 2: Sample velocities from the window
            samples = self.sample_velocities(v_range, w_range)
            if samples.size == 0:
                self.get_logger().warn("No valid samples in dynamic window.")
                return

            # Step 3: Predict all trajectories
            all_trajectories_np = self.predict_all_trajectories(samples)
            
            # Get data needed for evaluation
            obstacle_points_np = self.get_obstacles_in_base_link()
            goal_in_base_link = self.transform_goal_to_base_link()
            
            if goal_in_base_link is None:
                self.get_logger().warn("Could not transform goal, skipping cycle.")
                return

            # Step 4 & 5: Evaluate all trajectories and find the best command
            best_cmd, best_score, safe_trajs, unsafe_trajs = (
                self.evaluate_trajectories(
                    samples, all_trajectories_np, obstacle_points_np, goal_in_base_link
                )
            )

            # Publish the best command
            cmd_vel_msg = Twist()
            if best_cmd is not None:
                cmd_vel_msg.linear.x = best_cmd[0]
                cmd_vel_msg.angular.z = best_cmd[1]
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # Publish visualization markers
            self.publish_trajectories(safe_trajs, unsafe_trajs)

        except Exception as e:
            self.get_logger().error(f"DWA logic failed: {e}")
            self.cmd_vel_pub.publish(Twist())

    # DWA Helper Functions (The "Guts")

    def calculate_dynamic_window(self) -> tuple[list[float], list[float]]:
        """
        Calculates the dynamic window of reachable velocities.
        """
        v_current, w_current = self.current_velocity
        dt = self.timer_period

        # Calculate reachable velocities based on acceleration
        v_reachable = [
            v_current - self.max_lin_accel * dt,
            v_current + self.max_lin_accel * dt,
        ]
        w_reachable = [
            w_current - self.max_ang_accel * dt,
            w_current + self.max_ang_accel * dt,
        ]

        # Intersect reachable velocities with physical robot limits
        v_range = [
            max(self.min_v, v_reachable[0]),
            min(self.max_v, v_reachable[1]),
        ]
        w_range = [
            max(-self.max_w, w_reachable[0]),
            min(self.max_w, w_reachable[1]),
        ]
        return v_range, w_range

    def sample_velocities(self, v_range: list[float], w_range: list[float]) -> np.ndarray:
        """
        Generates a (N*M, 2) array of [v, w] samples using 'ij' indexing.
        """
        vv: np.ndarray
        ww: np.ndarray

        if v_range[0] > v_range[1] or w_range[0] > w_range[1]:
            return np.array([]) 

        v_space = np.linspace(v_range[0], v_range[1], self.v_samples)
        w_space = np.linspace(w_range[0], w_range[1], self.w_samples)
        
        # Create a 2D grid of all (v, w) combinations
        vv, ww = np.meshgrid(v_space, w_space, indexing='ij')
        
        # Stack into a (N*M, 2) array
        samples = np.stack([vv.ravel(), ww.ravel()], axis=1)
        
        return samples

    def evaluate_trajectories(self, samples: np.ndarray, all_trajectories_np: np.ndarray, obstacle_points_np: np.ndarray, goal_in_base_link) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
        """
        Vectorized evaluation of all trajectories for collision and scoring.
        """
        # Calculate collisions and min distances for all trajectories
        min_dists, is_collision = self._check_all_collisions_and_get_dists(
            all_trajectories_np, obstacle_points_np
        )
        
        # Calculate scores for all trajectories
        scores = self._score_all_trajectories(
            samples, all_trajectories_np, min_dists, goal_in_base_link
        )
        
        # Invalidate scores of trajectories that are in collision
        scores[is_collision] = -np.inf
        
        # Find the best command
        if np.all(scores == -np.inf):
            best_cmd = np.array([0.0, 0.0]) # No safe path, stop
            best_score = -np.inf
        else:
            best_index = np.argmax(scores)
            best_score = scores[best_index]
            best_cmd = samples[best_index]
        
        # Prepare lists for visualization
        safe_indices = ~is_collision
        unsafe_indices = is_collision
        safe_trajs = all_trajectories_np[safe_indices]
        unsafe_trajs = all_trajectories_np[unsafe_indices]

        return best_cmd, best_score, safe_trajs, unsafe_trajs

    def predict_all_trajectories(self, samples: np.ndarray) -> np.ndarray:
        """
        Simulates all (N) trajectories for (T) steps at once.
        Returns: (N, T, 3) np.ndarray of [x, y, yaw] poses
        """
        N = samples.shape[0]
        T = self.predict_steps
        dt = self.dt

        v_all = samples[:, 0]
        w_all = samples[:, 1]

        trajectories_np = np.zeros((N, T, 3))
        current_poses = np.zeros((N, 3)) # [x, y, yaw] for all N trajectories

        # Loop over time steps (T), calculating all N trajectories in parallel
        for t in range(T):
            current_yaw = current_poses[:, 2]

            # Update poses using simple kinematics
            new_yaw = current_yaw + w_all * dt
            new_x = current_poses[:, 0] + v_all * np.cos(new_yaw) * dt
            new_y = current_poses[:, 1] + v_all * np.sin(new_yaw) * dt
            
            # Store new poses
            current_poses[:, 0] = new_x
            current_poses[:, 1] = new_y
            current_poses[:, 2] = new_yaw
            
            trajectories_np[:, t, :] = current_poses
        
        return trajectories_np

    def _score_all_trajectories(self, samples: np.ndarray, all_trajectories_np: np.ndarray, min_dists: np.ndarray, goal_in_base_link: tuple[float, float]) -> np.ndarray:
        """
        Vectorized scoring of all trajectories.
        Returns: (N,) np.ndarray of scores (higher is better)
        """
        goal_x, goal_y = goal_in_base_link
        N = samples.shape[0]
        
        # --- Goal Cost ---
        # Get the final pose for all trajectories
        final_poses = all_trajectories_np[:, -1, :]
        final_x = final_poses[:, 0]
        final_y = final_poses[:, 1]
        final_yaw = final_poses[:, 2]
        
        # Get the angle from the end of each path to the goal
        angle_to_goal = np.arctan2(goal_y - final_y, goal_x - final_x)
        heading_error = np.abs(np.clip((angle_to_goal - final_yaw), -2 * np.pi, 2 * np.pi))
        goal_cost = heading_error / pi # Normalize cost to [0, 1]

        # --- Obstacle Cost ---
        # Cost is inverse of distance, penalizing proximity
        obstacle_cost = 1.0 / np.maximum(min_dists, 0.01) # 0.01 to avoid div by zero

        # --- Velocity Cost ---
        # Cost is normalized inverse of linear velocity, penalizing slowness
        v_all = samples[:, 0]
        velocity_cost = (self.max_v - v_all) / self.max_v

        # --- Total Score ---
        total_cost = (
            (self.alpha * goal_cost)
            + (self.beta * obstacle_cost)
            + (self.gamma * velocity_cost)
        )
        
        # Return negative cost because we want to maximize score
        return -total_cost

    # TF & Utility Functions

    def _check_all_collisions_and_get_dists(self, all_trajectories_np: np.ndarray, obstacle_points_np: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates collisions and min distances for all trajectories at once.
        Returns: tuple (min_dists (N,), is_collision (N,))
        """
        N = all_trajectories_np.shape[0]

        # If no obstacles, all paths are safe with infinite distance
        if obstacle_points_np.size == 0:
            min_dists = np.full((N,), float('inf'))
            is_collision = np.zeros((N,), dtype=bool)
            return min_dists, is_collision

        if all_trajectories_np.size == 0:
            return np.array([]), np.array([])
            
        # Get (x, y) coordinates from trajectories
        traj_xy = all_trajectories_np[:, :, :2] # Shape (N, T, 2)

        # Use broadcasting to find all distances at once
        # (N, T, 1, 2) - (1, 1, O, 2) -> (N, T, O, 2)
        diff = traj_xy[:, :, np.newaxis, :] - obstacle_points_np[np.newaxis, np.newaxis, :, :]
        
        # Calculate squared distances (N, T, O)
        dist_sq = np.sum(diff**2, axis=3)
        
        # Find min squared distance for each trajectory (N,)
        min_dist_sq_per_traj = np.min(dist_sq, axis=(1, 2))
        
        min_dists = np.sqrt(min_dist_sq_per_traj)
        is_collision = min_dists < self.robot_radius

        return min_dists, is_collision

    def get_obstacles_in_base_link(self) -> np.ndarray:
        """
        Converts LaserScan to a (N, 2) np.ndarray of [x, y] points in 'base_link'.
        """
        if self.current_scan is None:
            return np.array([])

        # Get the transform from the laser frame to the robot's center
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.current_scan.header.frame_id, rclpy.time.Time()
            )
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return np.array([])
        
        # Create an array of all angles in the scan
        num_readings = len(self.current_scan.ranges)
        angles = np.linspace(
            self.current_scan.angle_min,
            self.current_scan.angle_max,
            num_readings
        )
        ranges_np = np.array(self.current_scan.ranges)

        # Filter out invalid range readings (inf, nan, 0)
        valid_indices = (ranges_np > self.current_scan.range_min) & \
                        (ranges_np < self.current_scan.range_max)
        valid_ranges = ranges_np[valid_indices]
        valid_angles = angles[valid_indices]

        # Convert valid points from polar to cartesian (relative to 'base_scan')
        scan_x = valid_ranges * np.cos(valid_angles)
        scan_y = valid_ranges * np.sin(valid_angles)

        # Translate points to the 'base_link' frame
        obstacle_x = scan_x + trans_x
        obstacle_y = scan_y + trans_y

        # Stack into a (N, 2) array of [x, y] points
        obstacle_points_np = np.stack([obstacle_x, obstacle_y], axis=1)
        
        return obstacle_points_np
    
    def transform_goal_to_base_link(self) -> tuple[float, float]:
        """
        Transforms the global goal (in 'odom') to the 'base_link' frame.
        """
        if self.current_goal is None or self.current_pose is None:
            return None
        
        goal_x_odom, goal_y_odom = self.current_goal
        robot_x, robot_y, robot_yaw = self.current_pose

        # 2D transformation math
        dx = goal_x_odom - robot_x
        dy = goal_y_odom - robot_y
        goal_x_base_link = dx * cos(-robot_yaw) - dy * sin(-robot_yaw)
        goal_y_base_link = dx * sin(-robot_yaw) + dy * cos(-robot_yaw)

        return [goal_x_base_link, goal_y_base_link]
    
    def is_goal_reached(self) -> bool:
        """
        Checks if the robot is within a tolerance radius of the goal.
        """
        if self.current_goal is None or self.current_pose is None:
            return False
        
        dist = hypot(
            self.current_goal[0] - self.current_pose[0],
            self.current_goal[1] - self.current_pose[1],
        )
        return dist < 0.2  # 20cm tolerance

    def get_yaw_from_quaternion(self, orientation: Quaternion) -> float:
        """
        Convert a geometry_msgs/Quaternion to a 2D yaw angle.
        """
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler[2]


    # Visualization

    def publish_trajectories(self, safe_trajs: np.ndarray, unsafe_trajs: np.ndarray):
        """
        Publishes all evaluated trajectories as a MarkerArray for RViz.
        """
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        
        # Add a DELETEALL marker to clear the previous set
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "base_link"
        delete_all_marker.header.stamp = stamp
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        # Add all safe (green) and unsafe (red) trajectories
        id_counter = 1
        for trajectory in safe_trajs:
            marker = self.create_trajectory_marker(trajectory, "base_link", stamp, id_counter, (0.0, 1.0, 0.0))
            marker_array.markers.append(marker)
            id_counter += 1

        for trajectory in unsafe_trajs:
            marker = self.create_trajectory_marker(trajectory, "base_link", stamp, id_counter, (1.0, 0.0, 0.0))
            marker_array.markers.append(marker)
            id_counter += 1
        
        self.marker_pub.publish(marker_array)

    def create_trajectory_marker(self, trajectory: list, frame_id: str, stamp: Time, marker_id: int, color_rgb: tuple[float, float, float]) -> Marker:
        """
        Helper function to create a single LINE_STRIP marker.
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.02
        
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 0.7
        
        marker.pose.orientation.w = 1.0

        # Add all [x, y] points from the trajectory
        for x, y, yaw in trajectory:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.01 # Float slightly above the ground plane
            marker.points.append(p)
            
        return marker

def main(args=None):
    rclpy.init(args=args)
    
    planner_node = None
    try:
        planner_node = DwaPlanner()
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if planner_node:
            planner_node.get_logger().fatal(f"Unhandled exception: {e}")
    finally:
        if planner_node:
            # On shutdown, stop the robot
            planner_node.cmd_vel_pub.publish(Twist())
            planner_node.get_logger().info("Shutting down DWA planner.")
            planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

