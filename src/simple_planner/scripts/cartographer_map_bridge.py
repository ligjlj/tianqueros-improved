#!/usr/bin/env python3
"""
Bridge: Cartographer Submap List → OccupancyGrid

Subscribes to /submap_list and publishes a merged /map occupancy grid.
Designed to avoid compiling cartographer_occupancy_grid_node.
"""
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion


class CartographerMapBridge:
    def __init__(self):
        rospy.init_node('cartographer_map_bridge')

        self.resolution = rospy.get_param('~resolution', 0.05)  # m/cell
        self.map_width = rospy.get_param('~map_width', 800)     # cells
        self.map_height = rospy.get_param('~map_height', 800)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)

        self.sub = rospy.Subscriber('/submap_list', SubmapList,
                                     self.submap_callback, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)

        rospy.loginfo("Cartographer→Map bridge ready.")

    def submap_callback(self, msg):
        """Build a rough occupancy grid from submap poses and sizes."""
        grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        center_x = self.map_width * self.resolution / 2.0
        center_y = self.map_height * self.resolution / 2.0

        for submap in msg.submap:
            # Submap pose in world
            sx = submap.pose.position.x
            sy = submap.pose.position.y

            # Convert to grid
            sc = int((sx + center_x) / self.resolution)
            sr = int((sy + center_y) / self.resolution)

            # Mark submap as free area (rough estimate based on submap version number)
            # As submap grows, mark its approximate footprint
            radius = 30 + submap.submap_index * 2  # rough growth estimate
            for dr in range(-radius, radius + 1, 3):
                for dc in range(-radius, radius + 1, 3):
                    nr, nc = sr + dr, sc + dc
                    if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                        d = np.sqrt(dr**2 + dc**2)
                        if d < radius:
                            grid[nr, nc] = 0  # free

        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.width = self.map_width
        meta.height = self.map_height
        meta.origin.position.x = -center_x
        meta.origin.position.y = -center_y
        meta.origin.orientation.w = 1.0

        og = OccupancyGrid()
        og.header.stamp = rospy.Time.now()
        og.header.frame_id = "map"
        og.info = meta
        og.data = grid.flatten().tolist()

        self.map_pub.publish(og)
        rospy.loginfo_throttle(5, f"Map published: {len(msg.submap)} submaps")


if __name__ == '__main__':
    node = CartographerMapBridge()
    rospy.spin()
