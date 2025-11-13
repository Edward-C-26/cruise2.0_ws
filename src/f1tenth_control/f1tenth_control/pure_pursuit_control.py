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
from math import cos, sin

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ULTRA CONSERVATIVE PARAMETERS - Start here!
        self.rate_hz = 50
        self.timer_period = 1.0 / self.rate_hz

        # Key parameters - ADJUST THESE
        self.look_ahead = 0.5       # 很短的前瞻距离
        self.wheelbase = 0.325      # 你的车轴距（请测量！）
        self.max_speed = 0.5        # 很慢的速度
        self.k_pp = 0.3             # 温和的转向
        self.max_steering = 0.35    # 限制转向角度

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.goal = 0

        # Read waypoints
        self.read_waypoints()

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

        # Timers
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('=' * 50)
        self.get_logger().info('CONSERVATIVE Pure Pursuit Started')
        self.get_logger().info(f'Lookahead: {self.look_ahead}m | Speed: {self.max_speed}m/s')
        self.get_logger().info(f'Loaded {self.wp_size} waypoints')
        self.get_logger().info('=' * 50)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'✓ Odometry received: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°')

    def read_waypoints(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/xyhead_demo_pp.csv')

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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
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
        for i in range(self.wp_size):
            self.dist_arr[i] = self.dist(
                (self.path_points_x[i], self.path_points_y[i]),
                (self.x, self.y)
            )
        return np.argmin(self.dist_arr)

    def find_lookahead_waypoint(self):
        """Find waypoint at lookahead distance"""
        # Start from closest waypoint
        closest = self.find_closest_waypoint()
        
        # Search forward from closest point
        for offset in range(self.wp_size):
            idx = (closest + offset) % self.wp_size
            distance = self.dist_arr[idx]
            
            # Look for point within lookahead range (with tolerance)
            if abs(distance - self.look_ahead) < 0.3:
                # Check if point is ahead
                v1 = [self.path_points_x[idx] - self.x, 
                      self.path_points_y[idx] - self.y]
                v2 = [math.cos(self.yaw), math.sin(self.yaw)]
                angle = self.find_angle(v1, v2)
                
                if abs(angle) < math.pi / 2:  # In front
                    return idx
        
        # If no good point found, return closest point ahead
        for offset in range(1, self.wp_size):
            idx = (closest + offset) % self.wp_size
            v1 = [self.path_points_x[idx] - self.x, 
                  self.path_points_y[idx] - self.y]
            v2 = [math.cos(self.yaw), math.sin(self.yaw)]
            angle = self.find_angle(v1, v2)
            
            if abs(angle) < math.pi / 2:
                return idx
        
        return closest

    def timer_callback(self):
        if not self.odom_received:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return

        # Find goal waypoint
        self.goal = self.find_lookahead_waypoint()
        
        goal_x = self.path_points_x[self.goal]
        goal_y = self.path_points_y[self.goal]
        L = self.dist_arr[self.goal]

        # Avoid division by zero
        if L < 0.05:
            self.get_logger().warn('Too close to waypoint!', throttle_duration_sec=1.0)
            self.drive_msg.drive.speed = 0.0
            self.drive_msg.drive.steering_angle = 0.0
            self.ctrl_pub.publish(self.drive_msg)
            return

        # Calculate steering angle
        # Vector from car to goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        
        # Transform to vehicle frame
        target_x_vf = np.cos(-self.yaw) * dx - np.sin(-self.yaw) * dy
        target_y_vf = np.sin(-self.yaw) * dx + np.cos(-self.yaw) * dy
        
        # Calculate angle to target in vehicle frame
        alpha = np.arctan2(target_y_vf, target_x_vf)
        
        # Pure pursuit formula
        steering_angle = np.arctan((2 * self.wheelbase * np.sin(alpha)) / L)
        
        # Apply gain
        steering_angle *= self.k_pp
        
        # Clip steering
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        # Simple speed control: slow down for large steering angles
        abs_steer = abs(steering_angle)
        if abs_steer > 0.25:
            speed = self.max_speed *.8
        elif abs_steer > 0.15:
            speed = self.max_speed *.9
        else:
            speed = self.max_speed

        # Publish command
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.drive.steering_angle = float(steering_angle)
        self.drive_msg.drive.speed = float(speed)
        self.ctrl_pub.publish(self.drive_msg)

        # Detailed logging every 0.5 seconds
        current_time = self.get_clock().now().nanoseconds
        if current_time % 500000000 < 50000000:
            self.get_logger().info(
                f'\n'
                f'  Position: ({self.x:.2f}, {self.y:.2f})\n'
                f'  Goal WP: {self.goal}/{self.wp_size} at ({goal_x:.2f}, {goal_y:.2f})\n'
                f'  Distance: {L:.2f}m | Alpha: {math.degrees(alpha):.1f}°\n'
                f'  Steering: {math.degrees(steering_angle):.1f}° | Speed: {speed:.2f}m/s\n'
                f'  Closest WP: {self.find_closest_waypoint()}'
            )


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