import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import hypot, atan2, cos, sin, pi

class DwaPlanner(Node):
    """
    A DWA local planner node for a differential drive robot (TurtleBot3).
    """

    def __init__(self):
        super().__init__('dwa_planner_node')

        # --- Parameters ---
        self.declare_parameters_and_load()

        # --- State Variables ---
        self.current_pose = None      # [x, y, yaw] in odom frame
        self.current_velocity = None  # [v, w]
        self.current_scan = None
        self.current_goal = None      # [x, y] in odom frame

        # --- TF (Coordinate Transforms) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 1. NEW: Define a QoS Profile for Visualization ---
        # This profile is "Transient Local" so that RViz can see the last
        # published message even if it starts after the node.
        marker_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Create Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2. NEW: Use the new QoS profile here
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/dwa_trajectories', 
            marker_qos_profile
        )

        # --- Create Subscribers ---
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        # --- The DWA "Engine" ---
        self.timer_period = 1.0 / self.controller_frequency
        self.create_timer(self.timer_period, self.run_dwa_logic)

        self.get_logger().info("DWA Planner Node initialized.")
        self.get_logger().info(
            f"Waiting for topics (odom, scan, goal_pose)..."
        )
    
    # --- Parameter Handling ---
    
    def declare_parameters_and_load(self):
        """
        Declare and load all DWA parameters from the YAML file.
        """
        # (Physics)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('robot_radius', 0.17)

        # (Prediction)
        self.declare_parameter('predict_time', 2.0)
        self.declare_parameter('time_step', 0.1)

        # (Sampling)
        self.declare_parameter('linear_samples', 11)
        self.declare_parameter('angular_samples', 21)
        self.declare_parameter('controller_frequency', 10.0)

        # (Scoring)
        self.declare_parameter('goal_cost_weight', 1.0)
        self.declare_parameter('obstacle_cost_weight', 2.0)
        self.declare_parameter('velocity_cost_weight', 0.1)

        # Load them
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

    # --- 1. Callbacks (Store Sensor Data) ---

    def odom_callback(self, msg: Odometry):
        self.current_velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        pose = msg.pose.pose
        yaw = self.get_yaw_from_quaternion(pose.orientation)
        self.current_pose = [pose.position.x, pose.position.y, yaw]

    def scan_callback(self, msg: LaserScan):
        self.current_scan = msg

    def goal_callback(self, msg: PoseStamped):
        # Goal is in 'odom' or 'map' frame, which we treat as our world frame
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New goal received: {self.current_goal}")

    # --- 2. The Main "Engine" Loop ---

    def run_dwa_logic(self):
        if (
            self.current_pose is None
            or self.current_scan is None
            or self.current_goal is None
            or self.current_velocity is None
        ):
            # Log this only once per second to avoid spam
            if self.get_clock().now().nanoseconds % 1e9 < 1e8:
                self.get_logger().warn(
                    "Waiting for topics (odom, scan, goal_pose)..."
                )
            return

        # Check if we've reached the goal
        if self.is_goal_reached():
            self.get_logger().info("Goal Reached!")
            # Stop the robot
            self.cmd_vel_pub.publish(Twist())
            # Clear the goal to stop planning
            self.current_goal = None
            # Clear trajectories from RViz
            self.publish_trajectories([], []) 
            return

        # --- DWA 5-Step Process ---
        try:
            # 1. Define Dynamic Window
            v_range, w_range = self.calculate_dynamic_window()

            # 2. Sample Velocities
            samples = self.sample_velocities(v_range, w_range)

            # Get obstacle points relative to the robot's center ('base_link')
            obstacle_points = self.get_obstacles_in_base_link()
            
            # Get goal position relative to the robot's center ('base_link')
            goal_in_base_link = self.transform_goal_to_base_link()
            if goal_in_base_link is None:
                self.get_logger().warn("Could not transform goal, skipping cycle.")
                return

            # 3. & 4. Predict and Score Trajectories
            best_cmd, best_score, safe_trajs, unsafe_trajs = (
                self.evaluate_trajectories(
                    samples, obstacle_points, goal_in_base_link
                )
            )

            # 5. Publish the Best Command
            cmd_vel_msg = Twist()
            if best_cmd is not None:
                cmd_vel_msg.linear.x = best_cmd[0]
                cmd_vel_msg.angular.z = best_cmd[1]
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # 6. (Assignment Req) Publish Visualization
            self.publish_trajectories(safe_trajs, unsafe_trajs)

        except Exception as e:
            self.get_logger().error(f"DWA logic failed: {e}")
            # Stop the robot on failure
            self.cmd_vel_pub.publish(Twist())

    # --- 3. DWA Helper Functions (The "Guts") ---

    def calculate_dynamic_window(self):
        """
        Calculates the dynamic window of reachable velocities
        based on current velocity and acceleration limits.
        """
        v_current, w_current = self.current_velocity
        dt = self.timer_period

        # 1. Reachable velocity window based on acceleration
        v_reachable = [
            v_current - self.max_lin_accel * dt,
            v_current + self.max_lin_accel * dt,
        ]
        w_reachable = [
            w_current - self.max_ang_accel * dt,
            w_current + self.max_ang_accel * dt,
        ]

        # 2. Intersect with physical robot limits
        v_range = [
            max(self.min_v, v_reachable[0]),
            min(self.max_v, v_reachable[1]),
        ]
        w_range = [
            max(-self.max_w, w_reachable[0]),
            min(self.max_w, w_reachable[1]),
        ]
        return v_range, w_range

    def sample_velocities(self, v_range, w_range):
        """
        Generates a grid of (v, w) samples within the dynamic window.
        """
        v_space = np.linspace(v_range[0], v_range[1], self.v_samples)
        w_space = np.linspace(w_range[0], w_range[1], self.w_samples)
        
        # Create a 2D grid of samples
        samples = []
        for v in v_space:
            for w in w_space:
                samples.append([v, w])
        return samples

    def evaluate_trajectories(self, samples, obstacle_points, goal_in_base_link):
        """
        Loop through all samples, predict, check collision, and score.
        """
        best_score = -float('inf')
        best_cmd = None
        safe_trajectories = []
        unsafe_trajectories = []

        for v, w in samples:
            # 1. Predict trajectory in 'base_link' frame
            trajectory = self.predict_trajectory(v, w)

            # 2. Check for collision
            is_collision = self.check_collision(trajectory, obstacle_points)

            if is_collision:
                unsafe_trajectories.append(trajectory)
                continue

            # 3. Score the safe trajectory
            score = self.score_trajectory(trajectory, obstacle_points, goal_in_base_link, v)
            safe_trajectories.append(trajectory)

            if score > best_score:
                best_score = score
                best_cmd = [v, w]
        
        if best_cmd is None:
            # If no safe path, just stop
            best_cmd = [0.0, 0.0]

        return best_cmd, best_score, safe_trajectories, unsafe_trajectories

    def predict_trajectory(self, v, w):
        """
        Simulates a single trajectory for 'predict_time' seconds.
        Returns a list of [x, y, yaw] poses in the 'base_link' frame.
        """
        trajectory = []
        x, y, yaw = 0.0, 0.0, 0.0  # Start at the center of 'base_link'

        for _ in range(self.predict_steps):
            # Simple kinematic model (Euler integration)
            yaw += w * self.dt
            x += v * cos(yaw) * self.dt
            y += v * sin(yaw) * self.dt
            trajectory.append([x, y, yaw])
        
        return trajectory

    def check_collision(self, trajectory, obstacle_points):
        """
        Checks if any point in the trajectory is too close to an obstacle.
        """
        for pose_x, pose_y, _ in trajectory:
            for obs_x, obs_y in obstacle_points:
                dist = hypot(pose_x - obs_x, pose_y - obs_y)
                if dist < self.robot_radius:
                    return True  # Collision
        return False  # Safe

    def score_trajectory(self, trajectory, obstacle_points, goal_in_base_link, v):
        """
        Scores a trajectory based on goal heading, obstacle clearance, and velocity.
        """
        # --- 1. Goal Cost (Heading) ---
        final_x, final_y, final_yaw = trajectory[-1]
        
        # Angle from path's end to the goal (in 'base_link')
        angle_to_goal = atan2(goal_in_base_link[1] - final_y, 
                              goal_in_base_link[0] - final_x)
        
        # Heading error is the difference between the robot's final angle
        # and the angle it *should* have to point at the goal.
        heading_error = abs(self.normalize_angle(angle_to_goal - final_yaw))
        # Normalize cost to 0-1 range (0 is best, pi is worst)
        goal_cost = heading_error / pi

        # --- 2. Obstacle Cost (Clearance) ---
        if not obstacle_points:
            min_dist = 10.0 # No obstacles, high clearance
        else:
            min_dist = float('inf')
            for pose_x, pose_y, _ in trajectory:
                for obs_x, obs_y in obstacle_points:
                    min_dist = min(min_dist, hypot(pose_x - obs_x, pose_y - obs_y))
        
        # Cost is inverse of distance. High cost if close.
        # Cap distance to avoid division by zero or huge costs
        obstacle_cost = 1.0 / max(min_dist, 0.01)

        # --- 3. Velocity Cost (Speed) ---
        # Cost is high if velocity is low
        velocity_cost = (self.max_v - v) / self.max_v

        # --- Total Score ---
        # We want to MINIMIZE cost. Our loop looks for MAX score.
        # So, we return the negative of the weighted sum of costs.
        total_cost = (
            (self.alpha * goal_cost)
            + (self.beta * obstacle_cost)
            + (self.gamma * velocity_cost)
        )

        return -total_cost

    # --- 4. TF & Utility Functions ---

    def get_obstacles_in_base_link(self):
        """
        Converts the latest LaserScan data into a list of (x, y)
        obstacle points in the 'base_link' frame.
        """
        if self.current_scan is None:
            return []

        obstacle_points = []
        try:
            # Get the transform from 'base_scan' (laser) to 'base_link' (robot center)
            # This is usually just a small static offset.
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.current_scan.header.frame_id, rclpy.time.Time()
            )
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return []

        angle = self.current_scan.angle_min
        for r in self.current_scan.ranges:
            if self.current_scan.range_min < r < self.current_scan.range_max:
                # 1. Convert polar (angle, r) to cartesian (x, y) in 'base_scan' frame
                scan_x = r * cos(angle)
                scan_y = r * sin(angle)
                
                # 2. Add the static transform to get (x, y) in 'base_link' frame
                obstacle_points.append([scan_x + trans_x, scan_y + trans_y])

            angle += self.current_scan.angle_increment
        
        return obstacle_points
    
    def transform_goal_to_base_link(self):
        """
        Transforms the global goal (in 'odom') to the 'base_link' frame.
        """
        if self.current_goal is None or self.current_pose is None:
            return None
        
        # Goal in odom frame
        goal_x_odom, goal_y_odom = self.current_goal
        
        # Robot pose in odom frame
        robot_x, robot_y, robot_yaw = self.current_pose

        # --- 2D Transform ---
        # 1. Translate goal relative to robot's position
        dx = goal_x_odom - robot_x
        dy = goal_y_odom - robot_y
        
        # 2. Rotate goal into robot's frame
        goal_x_base_link = dx * cos(-robot_yaw) - dy * sin(-robot_yaw)
        goal_y_base_link = dx * sin(-robot_yaw) + dy * cos(-robot_yaw)

        return [goal_x_base_link, goal_y_base_link]
    
    def is_goal_reached(self):
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

    def get_yaw_from_quaternion(self, orientation):
        """
        Convert a geometry_msgs/Quaternion to a 2D yaw angle.
        """
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    # --- 5. Visualization ---

    def publish_trajectories(self, safe_trajs, unsafe_trajs):
        """
        Publishes all evaluated trajectories as a MarkerArray for RViz.
        """
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        
        # 1. Add a "DELETEALL" marker to clear old trajectories
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "base_link"
        delete_all_marker.header.stamp = stamp
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        # 2. Add all safe trajectories (green)
        id_counter = 1  # <-- *** FIX IS HERE ***
        for trajectory in safe_trajs:
            marker = self.create_trajectory_marker(trajectory, "base_link", stamp, id_counter, (0.0, 1.0, 0.0))
            marker_array.markers.append(marker)
            id_counter += 1

        # 3. Add all unsafe trajectories (red)
        for trajectory in unsafe_trajs:
            marker = self.create_trajectory_marker(trajectory, "base_link", stamp, id_counter, (1.0, 0.0, 0.0))
            marker_array.markers.append(marker)
            id_counter += 1
        
        self.marker_pub.publish(marker_array)

    def create_trajectory_marker(self, trajectory, frame_id, stamp, marker_id, color_rgb):
        """
        Helper function to create a single LINE_STRIP marker.
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.02  # Line width
        
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 0.7  # Transparency
        
        marker.pose.orientation.w = 1.0 # Identity orientation

        # Add all points from the trajectory
        for x, y, yaw in trajectory:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.01  # Slightly above the ground
            marker.points.append(p)
            
        return marker


# --- Standard Python main() function ---
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
            # Stop the robot on shutdown
            planner_node.cmd_vel_pub.publish(Twist())
            planner_node.get_logger().info("Shutting down DWA planner.")
            planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

