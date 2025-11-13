#!/usr/bin/env python3
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
            [-1.7, 1.9],  # waypoint 0
            [-1.6, 1.7],  # waypoint 1
            [-1.4, 1.7],  # waypoint 2
            [-1.1, 1.9],  # waypoint 3
            [-1.0, 1.8],  # waypoint 4
            [-0.8, 1.9],  # waypoint 5
            [-0.6, 1.9],  # waypoint 6
            [-0.4, 1.9],  # waypoint 7
            [-0.2, 1.9],  # waypoint 8
            [-0.0, 1.9],  # waypoint 9
            [0.2, 1.9],  # waypoint 10
            [0.4, 1.9],  # waypoint 11
            [0.6, 1.9],  # waypoint 12
            [0.7, 1.9],  # waypoint 13
            [0.9, 1.9],  # waypoint 14
            [1.1, 1.8],  # waypoint 15
            [1.2, 1.8],  # waypoint 16
            [1.4, 1.8],  # waypoint 17
            [1.6, 1.8],  # waypoint 18
            [1.7, 1.8],  # waypoint 19
            [1.9, 1.8],  # waypoint 20
            [2.1, 1.8],  # waypoint 21
            [2.3, 1.8],  # waypoint 22
            [2.6, 1.9],  # waypoint 23
            [2.8, 1.8],  # waypoint 24
            [3.0, 1.9],  # waypoint 25
            [3.0, 2.1],  # waypoint 26
            [3.1, 2.3],  # waypoint 27
            [3.1, 2.6],  # waypoint 28
            [2.9, 2.7],  # waypoint 29
            [2.7, 2.7],  # waypoint 30
            [2.5, 2.7],  # waypoint 31
            [2.3, 2.7],  # waypoint 32
            [2.1, 2.8],  # waypoint 33
            [1.9, 2.8],  # waypoint 34
            [1.7, 2.8],  # waypoint 35
            [1.4, 2.8],  # waypoint 36
            [1.2, 2.7],  # waypoint 37
            [1.0, 2.7],  # waypoint 38
            [0.8, 2.7],  # waypoint 39
            [0.6, 2.7],  # waypoint 40
            [0.4, 2.7],  # waypoint 41
            [0.2, 2.7],  # waypoint 42
            [-0.1, 2.7],  # waypoint 43
            [-0.3, 2.7],  # waypoint 44
            [-0.5, 2.7],  # waypoint 45
            [-0.7, 2.7],  # waypoint 46
            [-0.9, 2.7],  # waypoint 47
            [-1.2, 2.7],  # waypoint 48
            [-1.4, 2.7],  # waypoint 49
            [-1.6, 2.8],  # waypoint 50
            [-1.9, 2.8],  # waypoint 51
            [-2.1, 2.7],  # waypoint 52
            [-2.2, 2.5],  # waypoint 53
            [-2.4, 2.1],  # waypoint 54
            [-2.1, 2.0],  # waypoint 55
            [-1.9, 1.8],  # waypoint 56
        ]
        
        # Track OUT (outer track)
        self.trackout_waypoints = [
            [-1.8, 1.4],  # waypoint 0
            [-1.5, 1.4],  # waypoint 1
            [-1.3, 1.4],  # waypoint 2
            [-1.1, 1.4],  # waypoint 3
            [-0.9, 1.4],  # waypoint 4
            [-0.7, 1.4],  # waypoint 5
            [-0.5, 1.4],  # waypoint 6
            [-0.3, 1.4],  # waypoint 7
            [-0.1, 1.4],  # waypoint 8
            [0, 1.4],  # waypoint 9
            [0.2, 1.4],  # waypoint 10
            [0.4, 1.4],  # waypoint 11
            [0.6, 1.4],  # waypoint 12
            [0.8, 1.4],  # waypoint 13
            [1.0, 1.4],  # waypoint 14
            [1.2, 1.4],  # waypoint 15
            [1.4, 1.4],  # waypoint 16
            [1.6, 1.4],  # waypoint 17
            [1.8, 1.4],  # waypoint 18
            [2.0, 1.4],  # waypoint 19
            [2.2, 1.4],  # waypoint 20
            [2.5, 1.4],  # waypoint 21
            [2.7, 1.4],  # waypoint 22
            [3.0, 1.5],  # waypoint 23
            [3.2, 1.6],  # waypoint 24
            [3.4, 1.8],  # waypoint 25
            [3.5, 2.1],  # waypoint 26
            [3.5, 2.3],  # waypoint 27
            [3.5, 2.5],  # waypoint 28
            [3.4, 2.7],  # waypoint 29
            [3.3, 3.1],  # waypoint 30
            [2.9, 3.1],  # waypoint 31
            [2.6, 3.2],  # waypoint 32
            [2.2, 3.2],  # waypoint 33
            [1.9, 3.2],  # waypoint 34
            [1.6, 3.2],  # waypoint 35
            [1.3, 3.2],  # waypoint 36
            [1.0, 3.2],  # waypoint 37
            [0.7, 3.2],  # waypoint 38
            [0.4, 3.2],  # waypoint 39
            [0.1, 3.2],  # waypoint 40
            [-0.2, 3.2],  # waypoint 41
            [-0.5, 3.2],  # waypoint 42
            [-0.8, 3.2],  # waypoint 43
            [-1.1, 3.2],  # waypoint 44
            [-1.3, 3.2],  # waypoint 45
            [-1.4, 3.2],  # waypoint 46
            [-1.6, 3.1],  # waypoint 47
            [-1.8, 3.0],  # waypoint 48
            [-2.0, 2.8],  # waypoint 49
            [-2.2, 2.5],  # waypoint 50
            [-2.5, 2.3],  # waypoint 51
            [-3.0, 2.1],  # waypoint 52
            [-2.8, 1.7],  # waypoint 53
            [-2.5, 1.5],  # waypoint 54
            [-2.2, 1.4],  # waypoint 55
        ]
    
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
