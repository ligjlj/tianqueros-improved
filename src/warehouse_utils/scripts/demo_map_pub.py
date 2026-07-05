#!/usr/bin/env python3
"""Demo: publish a warehouse-like occupancy grid for the navigation stack."""
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import sys

def create_warehouse_map(width=200, height=200, resolution=0.05):
    """Create a warehouse-like map with walls, aisles, and obstacles."""
    grid = np.zeros((height, width), dtype=np.int8)
    
    # Outer walls
    grid[0:3, :] = 100
    grid[-3:, :] = 100
    grid[:, 0:3] = 100
    grid[:, -3:] = 100
    
    # Horizontal shelves (rows of obstacles with gaps)
    shelf_rows = [40, 80, 120, 160]
    for r in shelf_rows:
        grid[r:r+3, 20:80] = 100
        grid[r:r+3, 120:180] = 100  # aisle in the middle
    
    # Some scattered obstacles (pallets)
    grid[60:68, 50:58] = 100
    grid[140:148, 140:148] = 100
    grid[100:108, 20:28] = 100
    
    # Add unknown borders around the known area
    # (Leave as is — everything else is free=0)
    
    return grid.flatten().tolist()

def main():
    rospy.init_node('demo_map_publisher')
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    
    width, height = 200, 200
    resolution = 0.05
    data = create_warehouse_map(width, height, resolution)
    
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.info = MapMetaData()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin = Pose()
    msg.info.origin.position.x = -5.0
    msg.info.origin.position.y = -5.0
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    
    rospy.loginfo(f"Publishing warehouse map: {width}x{height} @ {resolution}m/cell")
    pub.publish(msg)
    rospy.spin()

if __name__ == '__main__':
    main()
