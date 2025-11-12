#!/usr/bin/env python3
"""
waypoint_list.py - F1TENTH Waypoint数据
存储两条赛道的waypoints
说明：
- Track IN: 内圈赛道
- Track OUT: 外圈赛道
- 使用 record_waypoints.py 录制后会自动更新此文件
"""

class WayPoints:
    """Waypoint manager"""
    
    def __init__(self):
        """Initialize waypoints"""
        
        # Track IN (inner track)
        self.trackin_waypoints = [
            # Example data (will be overwritten after recording)
            [0.0, -98.0],   # waypoint 0
            [1.0, -98.0],   # waypoint 1
            [2.0, -98.0],   # waypoint 2
            # ... use record_waypoints.py to record more
        ]
        
        # Track OUT (outer track)
        self.trackout_waypoints = [
            # Example data (will be overwritten after recording)
            [0.0, -90.0],   # waypoint 0
            [1.0, -90.0],   # waypoint 1
            [2.0, -90.0],   # waypoint 2
            # ... use record_waypoints.py to record more
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
    
    def get_num_waypoints(self):
        """Get waypoint counts"""
        return {
            'trackin': len(self.trackin_waypoints),
            'trackout': len(self.trackout_waypoints),
            'total': len(self.trackin_waypoints) + len(self.trackout_waypoints)
        }


if __name__ == '__main__':
    # Test code
    wp = WayPoints()
    stats = wp.get_num_waypoints()
    
    print("\n" + "="*60)
    print("Waypoint Statistics")
    print("="*60)
    print(f"Track IN:  {stats['trackin']} waypoints")
    print(f"Track OUT: {stats['trackout']} waypoints")
    print(f"Total:     {stats['total']} waypoints")
    print("="*60 + "\n")
    
    print("Track IN first 5 points:")
    for i, wp in enumerate(wp.get_trackin()[:5]):
        print(f"  [{i}] {wp}")
    
    print("\nTrack OUT first 5 points:")
    for i, wp in enumerate(wp.get_trackout()[:5]):
        print(f"  [{i}] {wp}")
