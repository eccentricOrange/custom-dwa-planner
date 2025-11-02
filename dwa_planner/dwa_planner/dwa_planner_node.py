import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf_transformations
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math

# ROS Message Imports
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class DwaPlanner(Node):
    """
    A ROS 2 node for implementing the Dynamic Window Approach (DWA)
    local planner from scratch.
    """

    def __init__(self):
        super().__init__('dwa_planner_node')
        self.get_logger().info("DWA Planner Node Started")

        # --- DWA Parameters ---
        # (You will need to tune these)
        
        # Robot physical constraints
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('min_linear_speed', 0.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0) # rad/s
        self.declare_parameter('max_linear_accel', 1.0)  # m/s^2
        self.declare_parameter('max_angular_accel', 3.5) # rad/s^2
        
        # Trajectory simulation parameters
        self.declare_parameter('predict_time', 2.0)      # seconds
        self.declare_parameter('time_step', 0.1)         # simulation time step, s
        self.declare_parameter('num_velocity_samples', 11) # Number of linear velocities to sample
        self.declare_parameter('num_angle_samples', 21)    # Number of angular velocities to sample

        # Cost function weights (alpha, beta, gamma)
        self.declare_parameter('goal_cost_weight', 1.0)
        self.declare_parameter('obstacle_cost_weight', 2.0)
        self.declare_parameter('velocity_cost_weight', 0.1)
        
        # Robot footprint / collision checking
        self.declare_parameter('robot_radius', 0.17) # meters

        # --- Get parameters ---
        self.max_v = self.get_parameter('max_linear_speed').value
        self.min_v = self.get_parameter('min_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        self.predict_time = self.get_parameter('predict_time').value
        self.dt = self.get_parameter('time_step').value
        self.v_samples = self.get_parameter('num_velocity_samples').value
        self.w_samples = self.get_parameter('num_angle_samples').value
        self.alpha = self.get_parameter('goal_cost_weight').value
        self.beta = self.get_parameter('obstacle_cost_weight').value
        self.gamma = self.get_parameter('velocity_cost_weight').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # --- Internal State Variables ---
        self.current_pose = None      # [x, y, yaw] in 'odom' frame
        self.current_velocity = None  # [v, w]
        self.current_scan = None      # LaserScan message
        self.current_goal = None      # [x, y] in 'odom' frame
        
        # --- TF (Coordinate Transforms) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.get_logger().info("Publishers and Subscribers successfully initialized.")

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # --- Main DWA Timer ---
        # This timer is the heart of the planner.
        self.dwa_timer = self.create_timer(0.1, self.run_dwa_logic) # 10 Hz
        self.get_logger().info("DWA timer started.")

    # --- 1. CALLBACKS (to update state) ---

    def odom_callback(self, msg):
        """Stores the robot's current pose and velocity."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        self.current_pose = [x, y, yaw]
        self.current_velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]

    def scan_callback(self, msg):
        """Stores the latest laser scan data."""
        self.current_scan = msg

    def goal_callback(self, msg):
        """Stores the latest goal pose."""
        # We only care about the 2D goal position
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New goal received: {self.current_goal}")

    # --- 2. THE DWA "ENGINE" (Main Loop) ---

    def run_dwa_logic(self):
        """
        The main DWA logic loop. Runs at a fixed rate.
        """
        # --- Safety Checks ---
        if self.current_pose is None or self.current_velocity is None or \
           self.current_scan is None or self.current_goal is None:
            self.get_logger().warn("Waiting for topics (odom, scan, goal_pose)...")
            return

        # --- DWA 5-Step Process ---
        # 1. Calculate the Dynamic Window
        v_range, w_range = self.calculate_dynamic_window(self.current_velocity)
        
        # 2. Sample Velocities
        samples = self.sample_velocities(v_range, w_range)
        if not samples:
            self.get_logger().warn("No valid velocity samples in dynamic window.")
            return

        # 3. & 4. Predict Trajectories and Score them
        # This is the core function you need to implement
        best_cmd, best_score, all_trajectories = self.evaluate_trajectories(samples)

        # 5. Publish the Best Command
        if best_cmd is None:
            self.get_logger().warn("No valid trajectory found. Stopping robot.")
            cmd_vel_msg = Twist() # Stop command
        else:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = best_cmd[0]
            cmd_vel_msg.angular.z = best_cmd[1]

        self.cmd_vel_pub.publish(cmd_vel_msg)

        # (Assignment Req) Publish Visualization
        self.publish_trajectories(all_trajectories)

    # --- 3. HELPER FUNCTIONS (The "Guts" you need to implement) ---

    def calculate_dynamic_window(self, current_vel):
        """
        Calculates the dynamic window (v_min, v_max, w_min, w_max)
        based on current velocity and acceleration limits.
        """
        v_c, w_c = current_vel
        
        # Calculate velocity range based on acceleration
        v_min_accel = v_c - self.max_lin_accel * self.dt
        v_max_accel = v_c + self.max_lin_accel * self.dt
        
        # Calculate angular velocity range based on acceleration
        w_min_accel = w_c - self.max_ang_accel * self.dt
        w_max_accel = w_c + self.max_ang_accel * self.dt

        # Combine with physical limits
        v_range = [
            max(self.min_v, v_min_accel),
            min(self.max_v, v_max_accel)
        ]
        w_range = [
            max(-self.max_w, w_min_accel),
            min(self.max_w, w_max_accel)
        ]
        
        return v_range, w_range

    def sample_velocities(self, v_range, w_range):
        """
        Generates a list of (v, w) samples from the dynamic window.
        """
        samples = []
        v_steps = np.linspace(v_range[0], v_range[1], self.v_samples)
        w_steps = np.linspace(w_range[0], w_range[1], self.w_samples)
        
        for v in v_steps:
            for w in w_steps:
                samples.append((v, w))
        return samples

    def evaluate_trajectories(self, samples):
        """
        Predicts, checks for collisions, and scores all trajectories.
        Returns the best (v, w) command.
        """
        best_score = -float('inf')
        best_cmd = None
        all_trajectories = [] # For visualization

        # Get laser scan points in the robot's 'base_link' frame
        obstacle_points = self.get_obstacles_in_base_link()
        if obstacle_points is None:
            self.get_logger().error("Could not get obstacle points. Stopping.")
            return None, -float('inf'), []

        for v, w in samples:
            # 3. Predict Trajectory
            # A trajectory is a list of [x, y, yaw] poses
            trajectory = self.predict_trajectory(v, w)
            
            # 4a. Check for Collisions
            if self.check_collision(trajectory, obstacle_points):
                # This trajectory is invalid, skip it
                all_trajectories.append((trajectory, 'red')) # For viz
                continue
            
            # 4b. Score the Valid Trajectory
            score = self.score_trajectory(trajectory, obstacle_points, v)
            all_trajectories.append((trajectory, 'green')) # For viz

            if score > best_score:
                best_score = score
                best_cmd = (v, w)
                
        return best_cmd, best_score, all_trajectories

    def predict_trajectory(self, v, w):
        """
        Simulates the robot's path for a given (v, w) for 'predict_time' seconds.
        Trajectory is in the robot's *own* frame ('base_link').
        """
        trajectory = []
        x, y, yaw = 0.0, 0.0, 0.0  # Start at the origin of the 'base_link' frame
        
        num_steps = int(self.predict_time / self.dt)
        
        for _ in range(num_steps):
            # Simple kinematic model (Euler integration)
            yaw += w * self.dt
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            trajectory.append([x, y, yaw])
            
        return trajectory

    def check_collision(self, trajectory, obstacle_points):
        """
        Checks if any point on the trajectory is too close to an obstacle.
        """
        if not obstacle_points:
            return False # No obstacles, no collision
            
        for pose in trajectory:
            traj_x, traj_y, _ = pose
            for obs_x, obs_y in obstacle_points:
                dist = math.hypot(traj_x - obs_x, traj_y - obs_y)
                if dist < self.robot_radius:
                    return True # Collision!
        return False # No collision

    def score_trajectory(self, trajectory, obstacle_points, v):
        """
        Calculates the score for a single valid trajectory.
        Uses the three cost functions (goal, obstacle, velocity).
        """
        # --- 1. Goal Cost (Heading) ---
        # How well does the final pose of the trajectory align with the goal?
        final_pose = trajectory[-1]
        
        # We need the goal in the 'base_link' frame
        goal_in_base_link = self.transform_goal_to_base_link()
        if goal_in_base_link is None:
            return -float('inf') # Can't score
            
        # Angle between robot's final heading and the goal
        goal_angle = math.atan2(goal_in_base_link[1] - final_pose[1],
                                goal_in_base_link[0] - final_pose[0])
        heading_error = abs(goal_angle - final_pose[2])
        
        # Normalize: cost is 0 (best) to pi (worst)
        goal_cost = heading_error
        
        # --- 2. Obstacle Cost (Clearance) ---
        # What is the *minimum* distance to an obstacle on this path?
        if not obstacle_points:
            min_dist = float('inf')
        else:
            min_dist = float('inf')
            for pose in trajectory:
                traj_x, traj_y, _ = pose
                for obs_x, obs_y in obstacle_points:
                    dist = math.hypot(traj_x - obs_x, traj_y - obs_y)
                    if dist < min_dist:
                        min_dist = dist
        
        # Cost is high if min_dist is small, 0 if it's large
        if min_dist < self.robot_radius:
             # This should not happen if check_collision is working
            return -float('inf')
        elif min_dist > self.robot_radius + 0.5: # 0.5m buffer
            obstacle_cost = 0.0
        else:
            obstacle_cost = 1.0 / min_dist # The closer, the higher the cost

        # --- 3. Velocity Cost (Speed) ---
        # Reward for moving at a good forward speed
        # Cost is 0 (best) to 1 (worst)
        velocity_cost = (self.max_v - v) / self.max_v
        
        # --- Total Score ---
        # We want to MINIMIZE cost, so we return the NEGATIVE weighted sum.
        # This way, a higher score is better.
        total_cost = (self.alpha * goal_cost) + \
                     (self.beta * obstacle_cost) + \
                     (self.gamma * velocity_cost)
                     
        return -total_cost

    # --- 4. UTILITY AND VISUALIZATION FUNCTIONS ---

    def get_obstacles_in_base_link(self):
        """
        Transforms the LaserScan data from its frame ('base_scan')
        to the robot's frame ('base_link') and returns a list of (x, y) points.
        """
        if self.current_scan is None:
            return None
        
        obstacle_points = []
        scan = self.current_scan
        
        try:
            # Get the transform from 'base_scan' to 'base_link'
            # This is usually just a static offset
            transform = self.tf_buffer.lookup_transform(
                'base_link', scan.header.frame_id, scan.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform 'base_scan' to 'base_link': {e}")
            return None

        for i, range_val in enumerate(scan.ranges):
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            
            # Convert polar (angle, range) to cartesian (x, y) in 'base_scan' frame
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            # Transform the point to 'base_link' frame
            # We create a dummy PoseStamped for the transformation
            p = PoseStamped()
            p.header.frame_id = scan.header.frame_id
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            
            # The built-in transform function doesn't exist for PoseStamped
            # in tf2_ros.py. We have to do the math manually.
            # This is complex. A simpler way for 2D:
            # (Note: This simple way assumes no rotation between base_scan and base_link, only translation)
            
            # A more robust (but complex) method involves quaternion math.
            # For this assignment, let's assume 'base_scan' and 'base_link'
            # have a simple translation offset from TF.
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
            
            obstacle_points.append((x + trans_x, y + trans_y))

        return obstacle_points

    def transform_goal_to_base_link(self):
        """
        Transforms the global goal (in 'odom' frame) to the 'base_link' frame.
        """
        if self.current_goal is None:
            return None
            
        try:
            # We need the transform from 'odom' to 'base_link'
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'odom', rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Apply the transform
            goal_x_odom, goal_y_odom = self.current_goal
            
            # Manual 2D transformation
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
            rot_q = transform.transform.rotation
            yaw = self.get_yaw_from_quaternion(rot_q)
            
            # Inverse transform: from world ('odom') to robot ('base_link')
            delta_x = goal_x_odom - self.current_pose[0]
            delta_y = goal_y_odom - self.current_pose[1]
            
            goal_x_base_link = (delta_x * math.cos(-self.current_pose[2]) - 
                                delta_y * math.sin(-self.current_pose[2]))
            goal_y_base_link = (delta_x * math.sin(-self.current_pose[2]) + 
                                delta_y * math.cos(-self.current_pose[2]))

            return [goal_x_base_link, goal_y_base_link]
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform 'odom' to 'base_link': {e}")
            return None

    def get_yaw_from_quaternion(self, quaternion):
        """Converts a geometry_msgs/Quaternion to a yaw angle (in radians)."""
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = tf_transformations.euler_from_quaternion(q)
        return euler[2]  # yaw

    def publish_trajectories(self, trajectories):
        """
        Publishes all evaluated trajectories as a MarkerArray for RViz.
        """
        marker_array = MarkerArray()
        
        # Delete all previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "base_link"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        marker_id = 0
        for traj_data in trajectories:
            trajectory, color_name = traj_data
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Size
            marker.scale.x = 0.02  # Line width
            
            # Color
            if color_name == 'red':
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            else:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            
            # Points
            for pose in trajectory:
                p = Point()
                p.x, p.y = pose[0], pose[1]
                p.z = 0.05 # Lift slightly off the ground
                marker.points.append(p)
                
            marker_array.markers.append(marker)
            marker_id += 1
            
        self.marker_pub.publish(marker_array)

# --- Standard Python Entry Point ---

def main(args=None):
    rclpy.init(args=args)
    try:
        planner_node = DwaPlanner()
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
