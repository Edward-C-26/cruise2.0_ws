#!/usr/bin/env python3
"""
track_switcher.py - 赛道切换算法
根据车辆状态计算最佳换道点，检查夹角、距离和前方曲率
"""

import math
import numpy as np


class TrackSwitcher:
    """Track switching algorithm - pure calculation, no motor control"""
    
    def __init__(self):
        """Initialize parameters"""
        
        # Angle constraint
        self.max_angle = math.radians(30)  # 30 degrees
        
        # Lookahead parameters
        self.lookahead_points = 5  # check 5 points ahead
        self.curvature_threshold = 0.15  # curvature > 0.15 means turning
        
        # Distance constraints
        self.min_distance = 2.0   # meters
        self.max_distance = 15.0  # meters
        
        # Prevent frequent switching
        self.last_switch_time = 0
        self.min_switch_interval = 5.0  # seconds
    
    def find_switch_point(self, vehicle_x, vehicle_y, vehicle_yaw, 
                          target_track, current_time):
        """
        Find the best switch point on target track
        
        Args:
            vehicle_x: vehicle X coordinate
            vehicle_y: vehicle Y coordinate
            vehicle_yaw: vehicle heading (radians)
            target_track: target track waypoints [[x,y], [x,y], ...]
            current_time: current timestamp (seconds)
        
        Returns:
            (waypoint, index) or (None, None)
        """
        
        # Check if switching is allowed (time interval)
        if current_time - self.last_switch_time < self.min_switch_interval:
            return None, None
        
        # Store qualified candidates
        candidates = []
        
        # Iterate through all waypoints in target track
        for idx, waypoint in enumerate(target_track):
            
            # Step 1: Lookahead check (is there a turn ahead?)
            if not self._check_straight_ahead(idx, target_track):
                continue  # turn detected, skip
            
            # Step 2: Calculate angle
            angle = self._calculate_angle(vehicle_x, vehicle_y, 
                                          vehicle_yaw, waypoint)
            
            if angle > self.max_angle:
                continue  # angle too large, skip
            
            # Step 3: Calculate distance
            distance = self._calculate_distance(vehicle_x, vehicle_y, 
                                                waypoint)
            
            if distance < self.min_distance or distance > self.max_distance:
                continue  # distance not suitable, skip
            
            # Qualified! Record it
            candidates.append({
                'waypoint': waypoint,
                'index': idx,
                'distance': distance,
                'angle': angle
            })
        
        # Step 4: Select the nearest point
        if not candidates:
            return None, None
        
        # Sort by distance, select nearest
        best = min(candidates, key=lambda x: x['distance'])
        
        # Record switch time
        self.last_switch_time = current_time
        
        return best['waypoint'], best['index']
    
    def _check_straight_ahead(self, start_idx, track_waypoints):
        """
        Lookahead check: is the path ahead straight (no turns)?
        
        Returns:
            True: straight ahead, can switch
            False: turn ahead, cannot switch
        """
        
        # Check if enough points available
        if start_idx + self.lookahead_points + 1 >= len(track_waypoints):
            return False
        
        # Extract lookahead path
        lookahead_path = track_waypoints[start_idx : start_idx + self.lookahead_points + 1]
        
        # Check curvature for every 3 consecutive points
        for i in range(len(lookahead_path) - 2):
            p1 = lookahead_path[i]
            p2 = lookahead_path[i + 1]
            p3 = lookahead_path[i + 2]
            
            curvature = self._calculate_curvature(p1, p2, p3)
            
            if curvature > self.curvature_threshold:
                # Turn detected!
                return False
        
        # All checks passed, straight path
        return True
    
    def _calculate_curvature(self, p1, p2, p3):
        """
        Calculate curvature formed by 3 points
        
        Returns:
            curvature value (rad/m), 0=straight, larger=more curved
        """
        
        # Calculate two vectors
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        # Vector lengths
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 0.01 or len2 < 0.01:
            return 0.0
        
        # Calculate angle between vectors
        cos_angle = np.dot(v1, v2) / (len1 * len2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # Curvature = angle / average distance
        avg_distance = (len1 + len2) / 2.0
        curvature = angle / avg_distance if avg_distance > 0 else 0.0
        
        return curvature
    
    def _calculate_angle(self, vehicle_x, vehicle_y, vehicle_yaw, waypoint):
        """
        Calculate angle between vehicle heading and waypoint direction
        
        Returns:
            angle (radians, 0 to pi)
        """
        
        # Calculate direction from vehicle to waypoint
        dx = waypoint[0] - vehicle_x
        dy = waypoint[1] - vehicle_y
        waypoint_direction = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle = waypoint_direction - vehicle_yaw
        
        # Normalize to -pi to pi
        angle = math.atan2(math.sin(angle), math.cos(angle))
        
        # Take absolute value
        angle = abs(angle)
        
        return angle
    
    def _calculate_distance(self, vehicle_x, vehicle_y, waypoint):
        """Calculate distance from vehicle to waypoint"""
        dx = waypoint[0] - vehicle_x
        dy = waypoint[1] - vehicle_y
        distance = math.sqrt(dx**2 + dy**2)
        return distance


if __name__ == '__main__':
    # Test code
    switcher = TrackSwitcher()
    
    # Mock vehicle state
    vehicle_x = 0.0
    vehicle_y = -98.0
    vehicle_yaw = 0.0  # heading east
    
    # Mock target track (straight section)
    target_track = [[i, -90.0] for i in range(20)]
    
    # Test
    waypoint, idx = switcher.find_switch_point(
        vehicle_x, vehicle_y, vehicle_yaw,
        target_track, 0.0
    )
    
    if waypoint:
        print(f"Found switch point: {waypoint} at index {idx}")
    else:
        print("No suitable switch point found")