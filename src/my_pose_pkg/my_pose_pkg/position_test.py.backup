#!/usr/bin/env python3
"""
position_test.py - F1TENTH位置测试工具
功能：订阅/ego_racecar/odom，实时打印车辆的xy坐标和朝向角度
使用：python3 position_test.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from util import quaternion_to_euler


class PositionTester(Node):
    """Position test node"""
    
    def __init__(self):
        super().__init__('position_tester')
        
        print("\n" + "="*60)
        print("F1TENTH Position Tester")
        print("="*60)
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        self.odom_received = False
        self.update_count = 0
        
        print("Waiting for data...\n")
    
    def odom_callback(self, msg):
        """Odometry callback"""
        
        if not self.odom_received:
            self.odom_received = True
            print("Data received!\n")
            print(f"{'Count':<8} {'X(m)':<12} {'Y(m)':<12} {'Yaw(deg)':<12} {'Speed(m/s)':<12}")
            print("-" * 60)
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = quaternion_to_euler([qx, qy, qz, qw])
        yaw_degrees = math.degrees(yaw)
        
        # Extract velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        
        # Print info (every 10 updates)
        self.update_count += 1
        if self.update_count % 10 == 0:
            print(f"{self.update_count:<8} {x:<12.3f} {y:<12.3f} {yaw_degrees:<12.2f} {speed:<12.3f}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    tester = PositionTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("Test finished!")
        print("="*60 + "\n")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
