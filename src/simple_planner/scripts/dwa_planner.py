#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) Local Planner.

Reads global plan (from A*) and outputs velocity commands to follow
it while avoiding obstacles detected in the occupancy grid.

Parameters:
  ~max_linear_vel   - max linear velocity (m/s, default 1.0)
  ~max_angular_vel  - max angular velocity (rad/s, default 1.5)
  ~predict_time     - trajectory simulation time (s, default 1.5)
  ~dt               - simulation step (s, default 0.1)
  ~robot_radius     - robot collision radius (m, default 0.5)
  ~goal_tolerance   - distance to goal to stop (m, default 0.3)
  ~alpha            - heading weight (default 0.5)
  ~beta             - clearance weight (default 2.0)
  ~gamma            - speed weight (default 0.3)
"""
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import euler_from_quaternion
import math


class DWAPlanner:
    def __init__(self):
        rospy.init_node('dwa_planner')

        # Parameters
        self.max_v = rospy.get_param('~max_linear_vel', 1.0)
        self.max_w = rospy.get_param('~max_angular_vel', 1.5)
        self.predict_time = rospy.get_param('~predict_time', 1.5)
        self.dt = rospy.get_param('~dt', 0.1)
        self.robot_radius = rospy.get_param('~robot_radius', 0.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        self.alpha = rospy.get_param('~alpha', 0.5)   # heading
        self.beta = rospy.get_param('~beta', 2.0)       # clearance
        self.gamma = rospy.get_param('~gamma', 0.3)     # speed

        # Velocity samples
        self.v_samples = rospy.get_param('~v_samples', 10)
        self.w_samples = rospy.get_param('~w_samples', 20)

        # State
        self.map_data = None
        self.map_info = None
        self.global_plan = None
        self.pose = None  # (x, y, theta)
        self.goal = None

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/planned_path', Path, self.path_cb, queue_size=1)

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.traj_marker_pub = rospy.Publisher('/dwa_trajectories', Marker, queue_size=1)

        rospy.loginfo("DWA Planner ready.")

    def map_cb(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_info = msg

    def path_cb(self, msg):
        self.global_plan = msg
        if msg.poses:
            last = msg.poses[-1].pose.position
            self.goal = (last.x, last.y)

    def set_pose(self, x, y, theta):
        self.pose = (x, y, theta)

    def world_to_grid(self, wx, wy):
        if self.map_info is None:
            return (0, 0)
        col = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return row, col

    def is_occupied(self, wx, wy):
        if self.map_data is None:
            return False
        r, c = self.world_to_grid(wx, wy)
        h, w = self.map_data.shape
        rr = int(self.robot_radius / self.map_info.resolution)
        for dr in range(-rr, rr + 1):
            for dc in range(-rr, rr + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if self.map_data[nr, nc] > 50:
                        return True
        if r < 0 or r >= h or c < 0 or c >= w:
            return True
        return False

    def simulate_trajectory(self, v, w, steps):
        x, y, theta = self.pose
        points = []
        for _ in range(steps):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            points.append((x, y, theta))
            if self.is_occupied(x, y):
                return points, True  # collision
        return points, False

    def score_trajectory(self, trajectory, collided):
        if collided or not trajectory:
            return -float('inf')

        last = trajectory[-1]
        x, y, theta = last

        # Heading: how well we point toward goal
        if self.goal:
            goal_heading = math.atan2(self.goal[1] - y, self.goal[0] - x)
            heading_error = abs(self._normalize_angle(goal_heading - theta))
            heading_score = (math.pi - heading_error) / math.pi
        else:
            heading_score = 1.0

        # Clearance: minimum distance to obstacles along trajectory
        min_dist = float('inf')
        for (tx, ty, _) in trajectory:
            # Check nearby cells
            if self.map_data is not None:
                r, c = self.world_to_grid(tx, ty)
                h, w = self.map_data.shape
                for dr in range(-5, 6):
                    for dc in range(-5, 6):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w:
                            if self.map_data[nr, nc] > 50:
                                d = math.sqrt(dr**2 + dc**2) * self.map_info.resolution
                                min_dist = min(min_dist, d)
        clearance_score = min(min_dist / self.robot_radius, 1.0) if min_dist != float('inf') else 1.0

        # Speed
        speed_score = abs(v) / self.max_v if self.max_v > 0 else 0

        return (self.alpha * heading_score +
                self.beta * clearance_score +
                self.gamma * speed_score)

    def find_best_velocity(self):
        if self.pose is None:
            return 0.0, 0.0

        steps = int(self.predict_time / self.dt)
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0
        all_traj = []

        for i in range(self.v_samples):
            v = self.max_v * i / (self.v_samples - 1) if self.v_samples > 1 else self.max_v
            for j in range(self.w_samples):
                w = -self.max_w + 2 * self.max_w * j / (self.w_samples - 1) if self.w_samples > 1 else 0.0
                traj, collided = self.simulate_trajectory(v, w, steps)
                score = self.score_trajectory(traj, collided)
                all_traj.append((traj, score))
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        # Publish best trajectory for viz
        self._publish_trajectories(all_traj)

        return best_v, best_w

    def _publish_trajectories(self, all_traj):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "dwa_traj"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.02
        marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.3)

        for traj, score in all_traj[:50]:  # limit to 50
            for i in range(len(traj) - 1):
                p1 = Point(x=traj[i][0], y=traj[i][1], z=0.05)
                p2 = Point(x=traj[i+1][0], y=traj[i+1][1], z=0.05)
                marker.points.extend([p1, p2])

        self.traj_marker_pub.publish(marker)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def run_demo(self):
        """Demonstration: simulate robot motion with DWA."""
        rate = rospy.Rate(10)

        # Simulate pose tracking (in real system, this comes from odometry)
        x, y, theta = 0.0, 0.0, 0.0

        while not rospy.is_shutdown():
            if self.goal is None:
                rate.sleep()
                continue

            self.set_pose(x, y, theta)

            # Check if goal reached
            if math.sqrt((x - self.goal[0])**2 + (y - self.goal[1])**2) < self.goal_tolerance:
                rospy.loginfo("Goal reached!")
                self.cmd_pub.publish(Twist())
                break

            # Find best velocity
            v, w = self.find_best_velocity()

            # Simulate motion
            x += v * math.cos(theta) * 0.1
            y += v * math.sin(theta) * 0.1
            theta += w * 0.1

            # Publish
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            # Publish local path
            lp = Path()
            lp.header.stamp = rospy.Time.now()
            lp.header.frame_id = "map"
            pose = PoseStamped()
            pose.header = lp.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            lp.poses.append(pose)
            self.local_path_pub.publish(lp)

            rospy.loginfo_throttle(2, f"[DWA] v={v:.2f} w={w:.2f} pos=({x:.1f},{y:.1f}) "
                                       f"goal_dist={math.sqrt((x-self.goal[0])**2+(y-self.goal[1])**2):.2f}")
            rate.sleep()


if __name__ == '__main__':
    planner = DWAPlanner()
    planner.run_demo()
