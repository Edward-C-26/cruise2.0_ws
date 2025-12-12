#!/usr/bin/env python3

import os
import csv
import math
import numpy as np
from numpy import linalg as la

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String  # ADDED THIS
from math import cos, sin
from std_msgs.msg import String, Bool


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ULTRA CONSERVATIVE PARAMETERS - Start here!
        self.rate_hz = 50
        self.timer_period = 1.0 / self.rate_hz

        # Key parameters - ADJUST THESE
        self.look_ahead = 0.6      # INCREASED: Helps prevent zig-zagging (0.8 to 1.2 is best)
        self.wheelbase = 0.325      
        self.max_speed = 0.5        # INCREASED: 0.5 is often too slow for smooth steering
        self.k_pp = 0.5            # GAIN: Lower this to 0.25 if it still oscillates
        self.max_steering = 0.45
        self.last_switch_time = 0
        self.detection_start_time = None # Timer for the reaction delay
        self.reaction_delay = 0
        self.safety_stop = False

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False # initiate to false, will be changed to true
        self.goal = 0

        # Lane state - starts with trackin (inner lane)
        self.current_lane = 'trackin'
        self.lane_files = {
            'trackin': '../../my_pose_pkg/my_pose_pkg/trackin_demo_pp.csv',
            'trackout': '../waypoints/xyhead_demo_pp.csv'
        }

        # Read waypoints
        self.read_waypoints(self.current_lane)

        # Publishers
        self.ctrl_pub = self.create_publisher(
            AckermannDriveStamped,
            "drive",
            10
        )
        self.path_pub = self.create_publisher(
            Path,
            'waypoints_path',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            'current_goal',
            10
        )
        self.lane_status_pub = self.create_publisher(
            String,
            'lane_status',
            10
        )
        self.lane_switch_sub = self.create_subscription(
            String,
            'lane_switch',
            self.lane_switch_callback,
            10
        )

        # Drive message
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "base_link"

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.lane_switch_sub = self.create_subscription(
            String,
            'lane_switch',
            self.lane_switch_callback,
            10
        )

        # --- NEW: Object Detection Subscriber ---
        # We use a different topic name to avoid conflicts
        self.object_sub = self.create_subscription(
            Bool,
            '/object_detected',  # The topic your detector node publishes to
            self.object_callback,
            10
        )

        # Timers
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('=' * 50)
        self.get_logger().info('CONSERVATIVE Pure Pursuit Started')
        self.get_logger().info(f'Lookahead: {self.look_ahead}m | Speed: {self.max_speed}m/s')
        self.get_logger().info(f'Current Lane: {self.current_lane}')
        self.get_logger().info(f'Loaded {self.wp_size} waypoints')
        self.get_logger().info('=' * 50)

    def object_callback(self, msg: Bool):
        """
        Toggle Lane Logic:
        If True received -> Switch to the OPPOSITE lane.
        Includes a 3-second cooldown so it doesn't spasm.
        """
        # 1. ONLY LISTEN TO "TRUE" SIGNALS
        if not msg.data:
            return

        # 2. CHECK COOLDOWN (Safety First)
        current_time = self.get_clock().now().nanoseconds
        # Check if 3 seconds have passed
        if (current_time - self.last_switch_time) < 3_000_000_000:
            return

        # 3. DETERMINE NEW LANE (Toggle)
        if self.current_lane == 'trackin':
            new_lane = 'trackout'
        else:
            new_lane = 'trackin'

        # 4. EXECUTE SWITCH
        self.get_logger().warn(f'OBSTACLE DETECTED! Dodging to {new_lane}!')
        self.switch_lane(new_lane)
        self.last_switch_time = current_time

        
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'✓ Odometry received: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°')

    def read_waypoints(self, lane_name='trackin'):
        """Read waypoints from specified lane file"""
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, self.lane_files[lane_name])

        if not os.path.exists(filename):
            self.get_logger().error(f'Waypoint file not found: {filename}')
            raise FileNotFoundError(f'Cannot find {filename}')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        self.path_points_x = np.array([float(point[0]) for point in path_points])
        self.path_points_y = np.array([float(point[1]) for point in path_points])
        self.wp_size = len(self.path_points_x)
        self.dist_arr = np.zeros(self.wp_size)
        
        # Print waypoint range for debugging
        self.get_logger().info(f'Waypoint X range: [{self.path_points_x.min():.2f}, {self.path_points_x.max():.2f}]')
        self.get_logger().info(f'Waypoint Y range: [{self.path_points_y.min():.2f}, {self.path_points_y.max():.2f}]')


    def publish_visualization(self):
        # Publish path
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(self.path_points_x, self.path_points_y):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # Publish current goal marker
        if self.odom_received:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'goal'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.path_points_x[self.goal])
            marker.pose.position.y = float(self.path_points_y[self.goal])
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            # Color based on current lane
            if self.current_lane == 'trackin':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            
            self.marker_pub.publish(marker)

    def dist(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def find_closest_waypoint(self):
        """Find the closest waypoint to current position"""
        # for i in range(self.wp_size):
        #     self.dist_arr[i] = self.dist(
        #         (self.path_points_x[i], self.path_points_y[i]),
        #         (self.x, self.y)
        #     )
        # return np.argmin(self.dist_arr)
        dx = self.path_points_x - self.x
        dy = self.path_points_y - self.y
        distances = np.hypot(dx, dy)
        return np.argmin(distances)

    def find_lookahead_waypoint(self):
        """
        ROBUST SEARCH: Finds the first point that is >= look_ahead distance away.
        """
        # 1. Start from the closest point (This fixes the 'NameError' you saw earlier)
        closest_idx = self.find_closest_waypoint()
        
        # 2. Search forward for a point far enough away
        for i in range(self.wp_size):
            idx = (closest_idx + i) % self.wp_size
            
            # Calculate actual distance to this point
            dx = self.path_points_x[idx] - self.x
            dy = self.path_points_y[idx] - self.y
            d = math.sqrt(dx**2 + dy**2)
            
            # If valid, return it
            if d >= self.look_ahead:
                return idx
        
        # Fallback: Just look 5 points ahead if all else fails
        return (closest_idx + 5) % self.wp_size

    # def find_lookahead_waypoint(self):
    #     """Find waypoint at lookahead distance"""
    #     # Start from closest waypoint
    #     closest = self.find_closest_waypoint()
        
    #     Search forward from closest point
    #     for offset in range(self.wp_size):
    #         idx = (closest + offset) % self.wp_size
    #         distance = self.dist_arr[idx]
            
    #         # Look for point within lookahead range (with tolerance)
    #         if abs(distance - self.look_ahead) < 0.3:
    #             # Check if point is ahead
    #             v1 = [self.path_points_x[idx] - self.x, 
    #                   self.path_points_y[idx] - self.y]
    #             v2 = [math.cos(self.yaw), math.sin(self.yaw)]
    #             angle = self.find_angle(v1, v2)
                
    #             if abs(angle) < math.pi / 2:  # In front
    #                 return idx
        
        
    #     If no good point found, return closest point ahead
    #     for offset in range(1, self.wp_size):
    #         idx = (closest + offset) % self.wp_size
    #         v1 = [self.path_points_x[idx] - self.x, 
    #               self.path_points_y[idx] - self.y]
    #         v2 = [math.cos(self.yaw), math.sin(self.yaw)]
    #         angle = self.find_angle(v1, v2)
            
    #         if abs(angle) < math.pi / 2:
    #             return idx
        
    #     return closest
      


    def timer_callback(self):

        if self.safety_stop:
            # Force stop command
            self.drive_msg.drive.speed = 0.0
            self.drive_msg.drive.steering_angle = 0.0  # Optional: Keep wheels straight
            self.ctrl_pub.publish(self.drive_msg)
            return  # Stop the function here so no other driving logic runs
        
        if not self.odom_received:
            # Only warn every 2 seconds to reduce spam
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return

        # --- 1. FIND GOALS ---
        # Find the point closest to the car (Where am I?)
        closest_wp_idx = self.find_closest_waypoint()
        
        # Find the point we want to drive to (Where am I going?)
        self.goal = self.find_lookahead_waypoint()
        
        goal_x = self.path_points_x[self.goal]
        goal_y = self.path_points_y[self.goal]
        
        # Calculate real distance to goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        L = math.sqrt(dx**2 + dy**2)

        # --- 2. CALCULATE STEERING ---
        # Transform goal to vehicle frame
        target_x_vf = np.cos(-self.yaw) * dx - np.sin(-self.yaw) * dy
        target_y_vf = np.sin(-self.yaw) * dx + np.cos(-self.yaw) * dy
        
        alpha = np.arctan2(target_y_vf, target_x_vf)
        steering_angle = np.arctan((2 * self.wheelbase * np.sin(alpha)) / L)
        
        # Apply gain and clip
        steering_angle *= self.k_pp
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        # --- 3. SPEED CONTROL ---
        # Smoothly slow down for sharp turns, but never stop completely
        # If steering is sharp (> 11 degrees), slow down to 80% speed
        if abs(steering_angle) > 0.2:
            speed = self.max_speed * 0.8
        else:
            speed = self.max_speed

        # Publish command
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.drive.steering_angle = float(steering_angle)
        self.drive_msg.drive.speed = float(speed)
        self.ctrl_pub.publish(self.drive_msg)

        # --- 4. DETAILED LOGGING (The print statements you asked for) ---
        # Only print 2 times per second to keep terminal clean
        # current_time = self.get_clock().now().nanoseconds
        # if current_time % 500000000 < 50000000:
        #     self.get_logger().info(
        #         f'\n------------------------------------------------\n'
        #         f'  CURRENT WP Index : {closest_wp_idx} / {self.wp_size}\n'
        #         f'  GOAL WP Index    : {self.goal}\n'
        #         f'  GOAL Coordinates : ({goal_x:.2f}, {goal_y:.2f})\n'
        #         f'  Car Speed        : {speed:.2f} m/s\n'
        #         f'------------------------------------------------'
        #     )
    # def timer_callback(self):
    #     if not self.odom_received:
    #         self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
    #         return

    #     # Find goal waypoint
    #     self.goal = self.find_lookahead_waypoint()
        
    #     goal_y = self.path_points_y[self.goal]
    #     L = self.dist_arr[self.goal]

    #     # Avoid division by zero
    #     if L < 0.05:
    #         self.get_logger().warn('Too close to waypoint!', throttle_duration_sec=1.0)
    #         self.drive_msg.drive.speed = 0.0
    #         self.drive_msg.drive.steering_angle = 0.0
    #         self.ctrl_pub.publish(self.drive_msg)
    #         return

    #     # Calculate steering angle
    #     # Vector from car to goal
    #     dx = goal_x - self.x
    #     dy = goal_y - self.y
        
    #     # Transform to vehicle frame
    #     target_x_vf = np.cos(-self.yaw) * dx - np.sin(-self.yaw) * dy
    #     target_y_vf = np.sin(-self.yaw) * dx + np.cos(-self.yaw) * dy
        
    #     # Calculate angle to target in vehicle frame
    #     alpha = np.arctan2(target_y_vf, target_x_vf)
        
    #     # Pure pursuit formula
    #     steering_angle = np.arctan((2 * self.wheelbase * np.sin(alpha)) / L)
        
    #     # Apply gain
    #     steering_angle *= self.k_pp
        
    #     # Clip steering
    #     steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
    #     # Simple speed control: slow down for large steering angles
    #     abs_steer = abs(steering_angle)
    #     if abs_steer > 0.25:
    #         speed = self.max_speed *.6
    #     elif abs_steer > 0.15:
    #         speed = self.max_speed *.75
    #     else:
    #         speed = self.max_speed

    #     # Publish command
    #     self.drive_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.drive_msg.drive.steering_angle = float(steering_angle)
    #     self.drive_msg.drive.speed = float(speed)
    #     self.ctrl_pub.publish(self.drive_msg)

    #     # Detailed logging every 0.5 seconds
    #     current_time = self.get_clock().now().nanoseconds
    #     if current_time % 500000000 < 50000000:
    #         self.get_logger().info(
    #             f'\n'
    #             f'  Lane: {self.current_lane}\n'
    #             f'  Position: ({self.x:.2f}, {self.y:.2f})\n'
    #             f'  Goal WP: {self.goal}/{self.wp_size} at ({goal_x:.2f}, {goal_y:.2f})\n'
    #             f'  Distance: {L:.2f}m | Alpha: {math.degrees(alpha):.1f}°\n'
    #             f'  Steering: {math.degrees(steering_angle):.1f}° | Speed: {speed:.2f}m/s\n'
    #             f'  Closest WP: {self.find_closest_waypoint()}'
    #         )

################################# LANE SWITCH FCN ##############################
    def lane_switch_callback(self, msg: String):
        """Handle lane switch commands"""
        requested_lane = msg.data.strip().lower()
        
        if requested_lane not in self.lane_files:
            self.get_logger().error(f'Invalid lane: {requested_lane}. Valid options: trackin, trackout')
            return
        
        if requested_lane == self.current_lane:
            self.get_logger().info(f'Already on {requested_lane} lane')
            return
        
        # Switch to new lane
        old_lane = self.current_lane
        self.switch_lane(requested_lane)
        self.get_logger().info(f'Lane switched: {old_lane} -> {requested_lane}')

    def switch_lane(self, lane_name):
        """Switch to a different lane waypoint file with directional matching"""
        if lane_name not in self.lane_files:
            self.get_logger().error(f'Lane {lane_name} not found!')
            return False
        
        try:
            # Store current state before switching
            current_goal_idx = self.goal
            current_target_x = self.path_points_x[current_goal_idx]
            current_target_y = self.path_points_y[current_goal_idx]
            
            # Calculate direction vector from current trajectory
            # Look ahead a few waypoints to get direction
            lookahead_points = min(5, self.wp_size - 1)
            next_idx = (current_goal_idx + lookahead_points) % self.wp_size
            direction_x = self.path_points_x[next_idx] - current_target_x
            direction_y = self.path_points_y[next_idx] - current_target_y
            direction_mag = math.sqrt(direction_x**2 + direction_y**2)
            
            if direction_mag > 0:
                direction_x /= direction_mag
                direction_y /= direction_mag
            
            self.get_logger().info(
                f'Current target: ({current_target_x:.2f}, {current_target_y:.2f}), '
                f'Direction: ({direction_x:.2f}, {direction_y:.2f})'
            )
            
            # Load new waypoints
            self.read_waypoints(lane_name)
            self.current_lane = lane_name
            
            # Find best matching waypoint considering both distance and direction
            best_score = float('-inf')
            best_idx = 0
            
            for i in range(self.wp_size):
                new_x = self.path_points_x[i]
                new_y = self.path_points_y[i]
                
                # Distance from old target to new candidate
                dist = self.dist((new_x, new_y), (current_target_x, current_target_y))
                
                # Direction vector at candidate waypoint (look ahead a few points)
                lookahead_new = min(5, self.wp_size - 1)
                next_new_idx = (i + lookahead_new) % self.wp_size
                new_dir_x = self.path_points_x[next_new_idx] - new_x
                new_dir_y = self.path_points_y[next_new_idx] - new_y
                new_dir_mag = math.sqrt(new_dir_x**2 + new_dir_y**2)
                
                if new_dir_mag > 0:
                    new_dir_x /= new_dir_mag
                    new_dir_y /= new_dir_mag
                
                # Dot product to measure direction alignment (-1 to 1)
                # 1 = same direction, -1 = opposite direction
                direction_alignment = direction_x * new_dir_x + direction_y * new_dir_y
                
                # Combined score: prioritize direction alignment, penalize distance
                # High positive score = good match
                # Weight direction heavily (×10) and penalize distance
                score = direction_alignment * 10.0 - dist * 0.5
                
                if score > best_score:
                    best_score = score
                    best_idx = i
            
            # Set the goal to the best matching waypoint
            self.goal = best_idx
            new_target_x = self.path_points_x[self.goal]
            new_target_y = self.path_points_y[self.goal]
            
            # Calculate alignment and distance for logging
            lookahead_final = min(5, self.wp_size - 1)
            next_new_idx = (best_idx + lookahead_final) % self.wp_size
            final_dir_x = self.path_points_x[next_new_idx] - new_target_x
            final_dir_y = self.path_points_y[next_new_idx] - new_target_y
            final_dir_mag = math.sqrt(final_dir_x**2 + final_dir_y**2)
            if final_dir_mag > 0:
                final_dir_x /= final_dir_mag
                final_dir_y /= final_dir_mag
            final_alignment = direction_x * final_dir_x + direction_y * final_dir_y
            
            final_dist = self.dist((new_target_x, new_target_y), (current_target_x, current_target_y))
            
            self.get_logger().info(
                f'New target waypoint: ({new_target_x:.2f}, {new_target_y:.2f})\n'
                f'  Cross-track distance: {final_dist:.2f}m\n'
                f'  Direction alignment: {final_alignment:.3f} (1.0=perfect)\n'
                f'  Waypoint index: {best_idx}/{self.wp_size}\n'
                f'  Score: {best_score:.2f}'
            )
            
            # Publish status
            status_msg = String()
            status_msg.data = f'Switched to {lane_name} lane'
            self.lane_status_pub.publish(status_msg)
            
            self.get_logger().info(f'✓ Successfully switched to {lane_name} lane')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to switch lane: {str(e)}')
            return False
##############################################################################


def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()

    try:
        rclpy.spin(pp)
    except KeyboardInterrupt:
        pp.get_logger().info('\nShutting down...')

    pp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()