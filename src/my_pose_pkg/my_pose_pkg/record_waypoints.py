#!/usr/bin/env python3
"""
record_waypoints.py - F1TENTH Waypoint录制工具
功能：手动驾驶车辆，按键录制两条赛道的waypoints
按键说明：
  SPACE   - 记录当前位置为waypoint
  T       - 切换车道（Track IN ↔ Track OUT）
  P       - 打印已录制的waypoints
  S       - 保存waypoints到文件
  Q       - 退出程序
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import os
import threading

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from util import quaternion_to_euler


class WaypointRecorder(Node):
    """Waypoint recorder node"""
    
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        print("\n" + "="*70)
        print("F1TENTH Waypoint Recorder")
        print("="*70)
        print("\nKeys:")
        print("  [SPACE]  - Record current position")
        print("  [T]      - Switch track (IN <-> OUT)")
        print("  [P]      - Print all waypoints")
        print("  [S]      - Save to waypoint_list.py")
        print("  [Q]      - Quit")
        print("\n" + "="*70 + "\n")
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        self.trackin_waypoints = []
        self.trackout_waypoints = []
        
        self.current_track = "IN"
        
        print(f"Current track: {self.current_track}")
        print("Waiting for odom data...\n")
    
    def odom_callback(self, msg):
        """Odometry callback"""
        
        if not self.odom_received:
            self.odom_received = True
            print("Odom received! Ready to record.\n")
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = quaternion_to_euler([qx, qy, qz, qw])
        self.current_yaw = yaw
    
    def record_waypoint(self):
        """Record current position"""
        
        if not self.odom_received:
            print("Waiting for odom data...")
            return
        
        x = round(self.current_x, 1)
        y = round(self.current_y, 1)
        waypoint = [x, y]
        
        if self.current_track == "IN":
            self.trackin_waypoints.append(waypoint)
            index = len(self.trackin_waypoints)
            track_name = "trackin_waypoints"
        else:
            self.trackout_waypoints.append(waypoint)
            index = len(self.trackout_waypoints)
            track_name = "trackout_waypoints"
        
        print(f"Recorded {track_name}[{index-1}] = [{x}, {y}]")
    
    def switch_track(self):
        """Switch track"""
        
        if self.current_track == "IN":
            self.current_track = "OUT"
        else:
            self.current_track = "IN"
        
        print("\n" + "="*70)
        print(f"Switched to Track {self.current_track}")
        
        if self.current_track == "IN":
            print(f"Track IN has {len(self.trackin_waypoints)} waypoints")
        else:
            print(f"Track OUT has {len(self.trackout_waypoints)} waypoints")
        
        print("="*70 + "\n")
    
    def print_waypoints(self):
        """Print all waypoints"""
        
        print("\n" + "="*70)
        print("Recorded Waypoints")
        print("="*70)
        
        print(f"\nTrack IN ({len(self.trackin_waypoints)} points):")
        print("-" * 70)
        if self.trackin_waypoints:
            for i, wp in enumerate(self.trackin_waypoints):
                print(f"  [{i:3d}] {wp}")
        else:
            print("  (empty)")
        
        print(f"\nTrack OUT ({len(self.trackout_waypoints)} points):")
        print("-" * 70)
        if self.trackout_waypoints:
            for i, wp in enumerate(self.trackout_waypoints):
                print(f"  [{i:3d}] {wp}")
        else:
            print("  (empty)")
        
        print("="*70 + "\n")
    
    def save_to_file(self):
        """Save waypoints to file"""
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(current_dir, 'waypoint_list.py')
        
        content = '''#!/usr/bin/env python3
"""
waypoint_list.py - F1TENTH Waypoint数据
自动生成，请勿手动编辑
"""

class WayPoints:
    """Waypoint manager"""
    
    def __init__(self):
        """Initialize waypoints"""
        
        # Track IN (inner track)
        self.trackin_waypoints = [
'''
        
        if self.trackin_waypoints:
            for i, wp in enumerate(self.trackin_waypoints):
                content += f"            {wp},  # waypoint {i}\n"
        else:
            content += "            # empty\n"
        
        content += '''        ]
        
        # Track OUT (outer track)
        self.trackout_waypoints = [
'''
        
        if self.trackout_waypoints:
            for i, wp in enumerate(self.trackout_waypoints):
                content += f"            {wp},  # waypoint {i}\n"
        else:
            content += "            # empty\n"
        
        content += '''        ]
    
    def get_trackin(self):
        """Get Track IN waypoints"""
        return self.trackin_waypoints
    
    def get_trackout(self):
        """Get Track OUT waypoints"""
        return self.trackout_waypoints
    
    def get_all_waypoints(self):
        """Get all waypoints (IN + OUT)"""
        return self.trackin_waypoints + self.trackout_waypoints


if __name__ == '__main__':
    wp = WayPoints()
    print(f"Track IN: {len(wp.get_trackin())} waypoints")
    print(f"Track OUT: {len(wp.get_trackout())} waypoints")
'''
        
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            print("\n" + "="*70)
            print("Saved successfully!")
            print(f"File: {file_path}")
            print(f"Track IN:  {len(self.trackin_waypoints)} waypoints")
            print(f"Track OUT: {len(self.trackout_waypoints)} waypoints")
            print("="*70 + "\n")
            
        except Exception as e:
            print(f"\nSave failed: {e}\n")
    
    def exit_program(self):
        """Exit program"""
        
        print("\n" + "="*70)
        print("Exiting...")
        print("="*70)
        
        self.print_waypoints()
        
        if self.trackin_waypoints or self.trackout_waypoints:
            self.save_to_file()
        
        print("Done!\n")
        rclpy.shutdown()


def keyboard_listener(recorder):
    """Keyboard listener thread"""
    
    print("Keyboard listener started\n")
    
    while rclpy.ok():
        try:
            key = input().strip().upper()
            
            if key == ' ' or key == '':
                recorder.record_waypoint()
            
            elif key == 'T':
                recorder.switch_track()
            
            elif key == 'P':
                recorder.print_waypoints()
            
            elif key == 'S':
                recorder.save_to_file()
            
            elif key == 'Q':
                recorder.exit_program()
                break
            
            else:
                print(f"Unknown command: '{key}'")
                print("Hint: SPACE=record, T=switch, P=print, S=save, Q=quit")
        
        except EOFError:
            break
        except KeyboardInterrupt:
            recorder.exit_program()
            break


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    recorder = WaypointRecorder()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()
    
    try:
        keyboard_listener(recorder)
    except KeyboardInterrupt:
        print("\n\nInterrupted...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
