#!/usr/bin/env python3
"""
RRT (Rapidly-exploring Random Tree) Path Planner on 2D Occupancy Grid

Subscribes:
  /map              — nav_msgs/OccupancyGrid
  /clicked_point    — geometry_msgs/PointStamped (goal from rviz)

Publishes:
  /planned_path     — nav_msgs/Path
  /path_marker      — visualization_msgs/Marker
  /rrt_tree         — visualization_msgs/Marker (tree edges, debug)

Parameters:
  ~step_size        — max extension step in cells (default 15)
  ~max_iter         — max iterations (default 5000)
  ~goal_sample_rate  — probability of sampling goal (default 0.1)
  ~robot_radius     — robot radius in cells (default 2)
  ~goal_tolerance   — goal reach tolerance in cells (default 5)
"""
import rospy
import numpy as np
import random
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header


class RRTPlanner:
    def __init__(self):
        rospy.init_node('rrt_planner')

        self.step_size = rospy.get_param('~step_size', 15)
        self.max_iter = rospy.get_param('~max_iter', 5000)
        self.goal_sample_rate = rospy.get_param('~goal_sample_rate', 0.1)
        self.robot_radius = rospy.get_param('~robot_radius', 2)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 5)

        self.map_data = None
        self.map_info = None

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=1)
        self.tree_pub = rospy.Publisher('/rrt_tree', Marker, queue_size=1)

        rospy.loginfo("RRT Planner ready. Click '2D Nav Goal' in rviz.")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_info = msg.info

    def goal_callback(self, msg):
        if self.map_info is None:
            rospy.logwarn("No map yet.")
            return

        goal = self._world_to_grid(msg.point.x, msg.point.y)
        start = (self.map_data.shape[0] // 2, self.map_data.shape[1] // 2)

        if not self._is_free(*goal):
            rospy.logwarn("Goal in obstacle.")
            return

        rospy.loginfo(f"RRT planning from {start} to {goal}")
        path = self.rrt_search(start, goal)

        if path:
            self.publish_path(path)
            rospy.loginfo(f"RRT path found! {len(path)} points, "
                          f"length={self._path_length(path):.2f}m")
        else:
            rospy.logwarn("RRT: no path found (increase max_iter or check map).")

    def rrt_search(self, start, goal):
        """Bidirectional RRT search."""
        nodes = [start]    # list of (row, col)
        parent = {start: None}

        for iteration in range(self.max_iter):
            # Sample
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._random_sample()

            # Find nearest
            nearest, nearest_idx = self._nearest(nodes, sample)

            # Extend
            new_node = self._extend(nearest, sample)

            if new_node is None:
                continue

            nodes.append(new_node)
            parent[new_node] = nearest

            # Publish tree edges periodically
            if iteration % 100 == 0:
                self._publish_tree(nodes, parent)

            # Check if goal reached
            dx = new_node[0] - goal[0]
            dy = new_node[1] - goal[1]
            if np.sqrt(dx**2 + dy**2) < self.goal_tolerance:
                # Reconstruct path
                path = [new_node]
                while path[-1] in parent and parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                path.reverse()
                self._publish_tree(nodes, parent)
                return path

        rospy.logwarn(f"RRT: max iterations ({self.max_iter}) reached.")
        self._publish_tree(nodes, parent)
        return None

    def _random_sample(self):
        h, w = self.map_data.shape
        return (random.randint(0, h - 1), random.randint(0, w - 1))

    def _nearest(self, nodes, target):
        best = None
        best_idx = -1
        best_dist = float('inf')
        for i, node in enumerate(nodes):
            d = (node[0] - target[0])**2 + (node[1] - target[1])**2
            if d < best_dist:
                best_dist = d
                best = node
                best_idx = i
        return best, best_idx

    def _extend(self, fr, to):
        dx = to[1] - fr[1]
        dy = to[0] - fr[0]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 1e-6:
            return None

        step = min(self.step_size, dist)
        ratio = step / dist
        new_c = int(fr[1] + dx * ratio)
        new_r = int(fr[0] + dy * ratio)

        if not self._is_free(new_r, new_c):
            return None

        # Check line segment for collision
        steps = int(step)
        for s in range(1, steps):
            c = int(fr[1] + dx * s / steps)
            r = int(fr[0] + dy * s / steps)
            if not self._is_free(r, c):
                return None

        return (new_r, new_c)

    def _is_free(self, row, col):
        h, w = self.map_data.shape
        if row < 0 or row >= h or col < 0 or col >= w:
            return False
        if self.map_data[row, col] == -1:  # unknown
            return True
        if self.map_data[row, col] > 50:
            return False
        # Check radius
        r = self.robot_radius
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if self.map_data[nr, nc] > 50:
                        return False
        return True

    def _world_to_grid(self, wx, wy):
        col = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return (row, col)

    def _grid_to_world(self, row, col):
        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def publish_path(self, grid_path):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_info.header.frame_id

        for (row, col) in grid_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = self._grid_to_world(row, col)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

        # Waypoint markers
        marker = Marker()
        marker.header = msg.header
        marker.ns = "rrt_waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = self.map_info.resolution * 3
        marker.color = ColorRGBA(1.0, 0.5, 0.0, 0.8)

        step = max(1, len(grid_path) // 100)
        for i in range(0, len(grid_path), step):
            p = Point()
            p.x, p.y = self._grid_to_world(*grid_path[i])
            p.z = 0.15
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def _publish_tree(self, nodes, parent):
        """Publish RRT tree edges for visualization."""
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.map_info.header.frame_id
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.map_info.resolution * 0.5
        marker.color = ColorRGBA(0.3, 0.6, 1.0, 0.3)

        count = 0
        for node in nodes:
            p = parent.get(node)
            if p is None:
                continue
            count += 1
            p1 = Point()
            p1.x, p1.y = self._grid_to_world(*p)
            p1.z = 0.05
            p2 = Point()
            p2.x, p2.y = self._grid_to_world(*node)
            p2.z = 0.05
            marker.points.extend([p1, p2])

        self.tree_pub.publish(marker)

    @staticmethod
    def _path_length(path):
        total = 0.0
        for i in range(1, len(path)):
            dr = path[i][0] - path[i-1][0]
            dc = path[i][1] - path[i-1][1]
            total += np.sqrt(dr**2 + dc**2)
        return total

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = RRTPlanner()
    planner.run()
