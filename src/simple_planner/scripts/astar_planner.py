#!/usr/bin/env python3
"""
A* Path Planner on 2D Occupancy Grid

Subscribes:
  /map              — nav_msgs/OccupancyGrid
  /clicked_point    — geometry_msgs/PointStamped (goal from rviz)

Publishes:
  /planned_path     — nav_msgs/Path (for rviz visualization)
  /path_marker      — visualization_msgs/Marker (waypoints)

Parameters:
  ~robot_radius     — robot radius in cells (default 2)
  ~heuristic_weight — A* heuristic weight (default 1.0, 1.0 = optimal)

Usage:
  rosrun simple_planner astar_planner.py
  # In rviz: click "2D Nav Goal" to set the goal, robot assumed at grid center initially
"""
import rospy
import numpy as np
from heapq import heappush, heappop
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
import tf


class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        # Parameters
        self.robot_radius = rospy.get_param('~robot_radius', 2)  # cells
        self.heuristic_weight = rospy.get_param('~heuristic_weight', 1.0)

        # State
        self.map_data = None
        self.map_info = None
        self.start = None  # (row, col) in grid

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.goal_callback, queue_size=1)

        # Publishers
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=1)

        rospy.loginfo("A* Planner ready. Click '2D Nav Goal' in rviz.")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info
        rospy.loginfo_throttle(10, f"Map received: {msg.info.width}x{msg.info.height}, "
                               f"res={msg.info.resolution:.3f}m")

    def goal_callback(self, msg):
        if self.map_info is None:
            rospy.logwarn("No map received yet. Cannot plan.")
            return

        # Convert goal from world coords to grid coords
        goal_col = int((msg.point.x - self.map_info.origin.position.x) /
                       self.map_info.resolution)
        goal_row = int((msg.point.y - self.map_info.origin.position.y) /
                       self.map_info.resolution)

        if not (0 <= goal_row < self.map_data.shape[0] and
                0 <= goal_col < self.map_data.shape[1]):
            rospy.logwarn(f"Goal ({goal_row},{goal_col}) is outside map boundaries.")
            return

        if self.map_data[goal_row, goal_col] > 50:
            rospy.logwarn(f"Goal cell is occupied (value={self.map_data[goal_row, goal_col]}).")
            return

        # Use map center as start if not specified
        if self.start is None:
            start_row = self.map_data.shape[0] // 2
            start_col = self.map_data.shape[1] // 2
            self.start = (start_row, start_col)

        rospy.loginfo(f"Planning A* from {self.start} to ({goal_row},{goal_col})")

        path = self.astar_search(self.start, (goal_row, goal_col))

        if path:
            self.publish_path(path)
            rospy.loginfo(f"Path found! {len(path)} waypoints, "
                          f"length={self.path_length(path):.2f}m")
        else:
            rospy.logwarn("No path found.")
            self.path_pub.publish(Path(header=Header(stamp=rospy.Time.now(),
                                       frame_id=self.map_info.header.frame_id)))

    def astar_search(self, start, goal):
        """A* search on the occupancy grid."""
        rows, cols = self.map_data.shape
        r = self.robot_radius

        def heuristic(r1, c1, r2, c2):
            dx = abs(r1 - r2)
            dy = abs(c1 - c2)
            return self.heuristic_weight * np.sqrt(dx**2 + dy**2)

        def is_free(row, col):
            if row < 0 or row >= rows or col < 0 or col >= cols:
                return False
            if self.map_data[row, col] == -1:  # unknown
                return True  # allow unknown
            if self.map_data[row, col] > 50:  # occupied
                return False
            # Check robot footprint
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if self.map_data[nr, nc] > 50:
                            return False
            return True

        def neighbors(row, col):
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = row + dr, col + dc
                if is_free(nr, nc):
                    cost = 1.414 if dr != 0 and dc != 0 else 1.0
                    yield nr, nc, cost

        # A* loop
        open_set = []
        heappush(open_set, (0.0 + heuristic(*start, *goal), 0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_set:
            _, g, current = heappop(open_set)

            if current in closed:
                continue

            if current == goal:
                # Reconstruct path
                path = [goal]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path

            closed.add(current)

            for nr, nc, cost in neighbors(*current):
                neighbor = (nr, nc)
                tentative_g = g + cost
                if neighbor in closed:
                    continue
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(nr, nc, *goal)
                    heappush(open_set, (f, tentative_g, neighbor))

        return None  # No path

    def publish_path(self, grid_path):
        """Convert grid path to ROS Path message and publish."""
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_info.header.frame_id

        for (row, col) in grid_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = (col * self.map_info.resolution +
                                    self.map_info.origin.position.x)
            pose.pose.position.y = (row * self.map_info.resolution +
                                    self.map_info.origin.position.y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

        # Also publish waypoint markers
        marker = Marker()
        marker.header = msg.header
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.map_info.resolution * 2
        marker.scale.y = self.map_info.resolution * 2
        marker.scale.z = self.map_info.resolution * 2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

        for (row, col) in grid_path[::max(1, len(grid_path) // 200)]:
            p = Point()
            p.x = col * self.map_info.resolution + self.map_info.origin.position.x
            p.y = row * self.map_info.resolution + self.map_info.origin.position.y
            p.z = 0.1
            marker.points.append(p)

        self.marker_pub.publish(marker)

    @staticmethod
    def path_length(path):
        total = 0.0
        for i in range(1, len(path)):
            dr = path[i][0] - path[i-1][0]
            dc = path[i][1] - path[i-1][1]
            total += np.sqrt(dr**2 + dc**2)
        return total

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = AStarPlanner()
    planner.run()
