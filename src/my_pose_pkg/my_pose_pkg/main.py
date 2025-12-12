#!/usr/bin/env python3
"""
main.py - F1TENTH主程序
循迹、换道控制，按空格切换赛道，跑完一圈停止
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time
import sys
import select
import termios
import tty

from waypoint_list import WayPoints
from track_switcher import TrackSwitcher
from util import quaternion_to_euler


class F1TenthController(Node):
    """Main controller with path tracking and lane switching"""
    
    def __init__(self):
        super().__init__('f1tenth_controller')
        
        print("\n" + "="*70)
        print("F1TENTH Controller - Path Tracking & Lane Switching")
        print("="*70)
        print("Press SPACE to switch track")
        print("Press Ctrl+C to exit")
        print("="*70 + "\n")
        
        # Load waypoints
        wp = WayPoints()
        self.trackin = wp.get_trackin()
        self.trackout = wp.get_trackout()
        
        # Current track state
        self.current_track = "IN"
        self.current_waypoints = self.trackin
        self.current_wp_index = 0
        
        # Track switcher
        self.switcher = TrackSwitcher()
        self.switch_requested = False
        
        # Vehicle state
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_speed = 0.0
        
        # Pure Pursuit parameters
        self.lookahead_distance = 2.0  # meters
        self.lookahead_points = 4      # number of points to lookahead
        
        # Speed control based on curvature
        self.base_speed = 2.0          # m/s
        self.max_speed = 3.0           # m/s
        self.min_speed = 1.0           # m/s
        
        # Lap completion detection
        self.lap_completed = False
        self.start_position = None
        self.lap_threshold = 3.0       # meters - distance to start for lap completion
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car3/odom',
            self.odom_callback,
            10
        )
        
        # Control timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Keyboard setup
        self.setup_keyboard()
        
        print("System initialized!")
        print(f"Current track: {self.current_track}")
        print(f"Waypoints loaded: IN={len(self.trackin)}, OUT={len(self.trackout)}\n")
    
    def setup_keyboard(self):
        """Setup non-blocking keyboard input"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    
    def cleanup_keyboard(self):
        """Restore keyboard settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def odom_callback(self, msg):
        """Update vehicle state from odometry"""
        
        # Extract position
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        
        # Extract orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        _, _, yaw = quaternion_to_euler([qx, qy, qz, qw])
        self.vehicle_yaw = yaw
        
        # Extract speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.vehicle_speed = math.sqrt(vx**2 + vy**2)
        
        # Record start position on first callback
        if self.start_position is None:
            self.start_position = [self.vehicle_x, self.vehicle_y]
            print(f"Start position recorded: [{self.vehicle_x:.2f}, {self.vehicle_y:.2f}]\n")
    
    def control_loop(self):
        """Main control loop - 20Hz"""
        
        # Check if lap completed
        if self.lap_completed:
            # MOTOR CONTROL: stop
            print("# STOP (speed=0.0 m/s)")
            return
        
        # Check keyboard input (non-blocking)
        if self.check_keyboard():
            self.switch_requested = True
            print("\n[SPACE] Switch requested!")
        
        # Handle track switching
        if self.switch_requested:
            self.handle_track_switch()
        
        # Check lap completion
        if self.check_lap_completion():
            self.lap_completed = True
            print("\n" + "="*70)
            print("LAP COMPLETED!")
            print("="*70 + "\n")
            # MOTOR CONTROL: stop
            print("# STOP (speed=0.0 m/s)")
            return
        
        # Pure Pursuit path tracking
        self.pure_pursuit_control()
    
    def handle_track_switch(self):
        """Handle track switching request"""
        
        # Determine target track
        if self.current_track == "IN":
            target_track = self.trackout
            target_name = "OUT"
        else:
            target_track = self.trackin
            target_name = "IN"
        
        # Find switch point
        waypoint, idx = self.switcher.find_switch_point(
            self.vehicle_x,
            self.vehicle_y,
            self.vehicle_yaw,
            target_track,
            time.time()
        )
        
        if waypoint is not None:
            # Switch successful
            print(f"\n{'='*70}")
            print(f"SWITCHING: Track {self.current_track} -> Track {target_name}")
            print(f"Switch point: {waypoint} at index {idx}")
            print(f"{'='*70}\n")
            
            # Update track
            self.current_waypoints = target_track
            self.current_wp_index = idx
            self.current_track = target_name
            
            # Reset switch request
            self.switch_requested = False
        else:
            print("[WARNING] No suitable switch point found, waiting...")
    
    def pure_pursuit_control(self):
        """Pure Pursuit algorithm with lookahead and curvature-based speed control"""
        
        if len(self.current_waypoints) == 0:
            print("[ERROR] No waypoints available")
            return
        
        # Find lookahead waypoint
        lookahead_wp, lookahead_idx = self.find_lookahead_waypoint()
        
        if lookahead_wp is None:
            print("[WARNING] No lookahead waypoint found")
            return
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(lookahead_wp)
        
        # Calculate path curvature for speed control
        curvature = self.calculate_path_curvature(lookahead_idx)
        
        # Adjust speed based on curvature
        target_speed = self.adjust_speed_by_curvature(curvature)
        
        # Update current waypoint index
        self.update_waypoint_index()
        
        # Execute motor control (commented)
        self.execute_motor_control(steering_angle, target_speed, curvature)
    
    def find_lookahead_waypoint(self):
        """Find waypoint at lookahead distance"""
        
        min_distance_diff = float('inf')
        best_waypoint = None
        best_idx = self.current_wp_index
        
        # Search ahead from current index
        search_range = min(len(self.current_waypoints), 
                          self.current_wp_index + self.lookahead_points + 5)
        
        for i in range(self.current_wp_index, search_range):
            wp = self.current_waypoints[i % len(self.current_waypoints)]
            
            # Calculate distance to waypoint
            distance = math.sqrt(
                (wp[0] - self.vehicle_x)**2 + 
                (wp[1] - self.vehicle_y)**2
            )
            
            # Find waypoint closest to lookahead distance
            distance_diff = abs(distance - self.lookahead_distance)
            if distance_diff < min_distance_diff:
                min_distance_diff = distance_diff
                best_waypoint = wp
                best_idx = i % len(self.current_waypoints)
        
        return best_waypoint, best_idx
    
    def calculate_steering_angle(self, target_waypoint):
        """Calculate steering angle to target waypoint"""
        
        # Vector from vehicle to target
        dx = target_waypoint[0] - self.vehicle_x
        dy = target_waypoint[1] - self.vehicle_y
        
        # Target heading
        target_heading = math.atan2(dy, dx)
        
        # Heading error
        heading_error = target_heading - self.vehicle_yaw
        
        # Normalize to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), 
                                   math.cos(heading_error))
        
        # Pure Pursuit formula: steering_angle = atan(2 * L * sin(alpha) / ld)
        # Simplified: steering_angle ≈ heading_error
        steering_angle = heading_error
        
        # Limit steering angle to [-30°, 30°]
        max_steering = math.radians(30)
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        return steering_angle
    
    def calculate_path_curvature(self, lookahead_idx):
        """Calculate curvature of upcoming path (next 4-5 points)"""
        
        if len(self.current_waypoints) < 3:
            return 0.0
        
        curvatures = []
        
        # Check curvature for next several points
        for i in range(self.lookahead_points):
            idx = (lookahead_idx + i) % len(self.current_waypoints)
            
            if idx + 2 >= len(self.current_waypoints):
                break
            
            p1 = self.current_waypoints[idx]
            p2 = self.current_waypoints[idx + 1]
            p3 = self.current_waypoints[idx + 2]
            
            curvature = self.switcher._calculate_curvature(p1, p2, p3)
            curvatures.append(curvature)
        
        # Return max curvature (most restrictive)
        return max(curvatures) if curvatures else 0.0
    
    def adjust_speed_by_curvature(self, curvature):
        """Adjust target speed based on path curvature"""
        
        # Speed control strategy:
        # - Straight path (low curvature): high speed
        # - Curved path (high curvature): low speed
        
        if curvature < 0.05:
            # Straight path
            speed = self.max_speed
        elif curvature < 0.15:
            # Gentle curve
            speed = self.base_speed
        elif curvature < 0.30:
            # Moderate curve
            speed = self.min_speed + 0.5
        else:
            # Sharp curve
            speed = self.min_speed
        
        return speed
    
    def update_waypoint_index(self):
        """Update current waypoint index based on proximity"""
        
        current_wp = self.current_waypoints[self.current_wp_index]
        distance = math.sqrt(
            (current_wp[0] - self.vehicle_x)**2 + 
            (current_wp[1] - self.vehicle_y)**2
        )
        
        # Move to next waypoint if close enough
        if distance < 0.5:  # meters
            self.current_wp_index = (self.current_wp_index + 1) % len(self.current_waypoints)
    
    def execute_motor_control(self, steering_angle, target_speed, curvature):
        """Execute motor control (commented for now)"""
        
        # Convert steering angle to degrees for readability
        steering_deg = math.degrees(steering_angle)
        
        # Determine turn direction
        if abs(steering_angle) < math.radians(5):
            direction = "STRAIGHT"
        elif steering_angle > 0:
            direction = "TURN LEFT"
        else:
            direction = "TURN RIGHT"
        
        # Acceleration/deceleration based on speed change
        speed_diff = target_speed - self.vehicle_speed
        if abs(speed_diff) < 0.1:
            accel_cmd = "maintain"
            accel_value = 0.0
        elif speed_diff > 0:
            accel_cmd = "accelerate"
            accel_value = min(speed_diff * 2.0, 2.0)  # max 2.0 m/s²
        else:
            accel_cmd = "decelerate"
            accel_value = max(speed_diff * 2.0, -3.0)  # max -3.0 m/s² (braking)
        
        # MOTOR CONTROL COMMANDS (commented)
        print(f"# {direction} (steering={steering_deg:+.1f}°, "
              f"speed={target_speed:.1f}m/s, {accel_cmd}={accel_value:+.2f}m/s², "
              f"curvature={curvature:.3f})")
        
        # Additional comments based on curvature
        if curvature > 0.3:
            print(f"#   -> Sharp turn detected! Slow down to {target_speed:.1f}m/s")
        elif curvature < 0.05:
            print(f"#   -> Straight path, speed up to {target_speed:.1f}m/s")
    
    def check_lap_completion(self):
        """Check if vehicle completed one lap"""
        
        if self.start_position is None:
            return False
        
        # Calculate distance to start position
        distance_to_start = math.sqrt(
            (self.vehicle_x - self.start_position[0])**2 + 
            (self.vehicle_y - self.start_position[1])**2
        )
        
        # Complete lap if close to start and traveled enough waypoints
        if (distance_to_start < self.lap_threshold and 
            self.current_wp_index > len(self.current_waypoints) * 0.8):
            return True
        
        return False
    
    def check_keyboard(self):
        """Check for keyboard input (non-blocking)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                return True
        return False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    controller = F1TenthController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Shutting down...")
        print("="*70 + "\n")
    finally:
        controller.cleanup_keyboard()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()