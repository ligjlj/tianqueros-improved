#!/usr/bin/env python3
"""
Autonomous Drone Controller — Waypoint Navigation

Subscribes:
  /drone/ground_truth  — nav_msgs/Odometry (drone pose from Gazebo)

Publishes:
  /drone/force          — geometry_msgs/Wrench (force/torque to Gazebo)
  /drone/trajectory     — nav_msgs/Path (for rviz)
"""
import rospy
import numpy as np
from collections import deque
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Wrench, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header


class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')

        # Waypoints (x, y, z)
        self.waypoints = [
            (4.0, 3.0, 2.0),     # WP1: fly to green marker
            (-2.0, -2.0, 2.5),   # WP2: second green marker
            (5.5, -1.5, 2.0),    # WP3: third green marker
            (0.0, 0.0, 2.0),     # Return to start
        ]
        self.current_wp_idx = 0
        self.mission_started = False

        # PID gains
        self.kp_xy = rospy.get_param('~kp_xy', 1.5)
        self.kd_xy = rospy.get_param('~kd_xy', 0.8)
        self.kp_z = rospy.get_param('~kp_z', 3.0)
        self.kd_z = rospy.get_param('~kd_z', 1.5)

        self.max_force_xy = rospy.get_param('~max_force_xy', 8.0)
        self.max_force_z = rospy.get_param('~max_force_z', 12.0)
        self.hover_thrust = rospy.get_param('~hover_thrust', 14.7)  # N (~1.5kg)

        # Waypoint tolerance
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.3)
        self.vel_tolerance = rospy.get_param('~vel_tolerance', 0.3)

        # State
        self.pose = None     # (x, y, z)
        self.vel = None      # (vx, vy, vz)
        self.prev_error = np.zeros(3)
        self.trajectory = deque(maxlen=500)

        # Subscribers
        rospy.Subscriber('/drone/ground_truth', Odometry,
                         self.pose_callback, queue_size=1)

        # Publishers
        self.force_pub = rospy.Publisher('/drone/force', Wrench, queue_size=1)
        self.traj_pub = rospy.Publisher('/drone/trajectory', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/drone/waypoint_markers', Marker,
                                          queue_size=1, latch=True)

        # Takeoff sequence
        self.state = "TAKEOFF"  # TAKEOFF → NAVIGATE → LAND → DONE
        self.hover_start = None
        self.takeoff_target = 2.0  # target altitude

        self.publish_waypoint_markers()
        rospy.loginfo("Drone controller ready. Starting autonomous mission...")

    def pose_callback(self, msg):
        self.pose = np.array([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z])
        self.vel = np.array([msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z])

        # Record trajectory
        self.trajectory.append(self.pose.copy())

    def publish_waypoint_markers(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

        for (x, y, z) in self.waypoints:
            p = Point(x=x, y=y, z=z)
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def compute_control(self, target, dt):
        """PID position controller → force vector."""
        if self.pose is None or self.vel is None:
            return np.zeros(3)

        error = target - self.pose
        derror = -self.vel  # target velocity is 0

        # PID
        gains_p = np.array([self.kp_xy, self.kp_xy, self.kp_z])
        gains_d = np.array([self.kd_xy, self.kd_xy, self.kd_z])

        force = gains_p * error + gains_d * derror

        # Clamp horizontal force
        f_xy = np.linalg.norm(force[:2])
        if f_xy > self.max_force_xy:
            force[:2] = force[:2] / f_xy * self.max_force_xy

        # Clamp vertical force
        force[2] = np.clip(force[2], -self.max_force_z + self.hover_thrust,
                           self.max_force_z + self.hover_thrust)

        # Add hover thrust
        force[2] += self.hover_thrust

        return force

    def publish_force(self, force):
        w = Wrench()
        w.force.x = force[0]
        w.force.y = force[1]
        w.force.z = force[2]
        # Small damping torque to prevent spinning
        if self.vel is not None:
            w.torque.x = -self.vel[3] if len(self.vel) > 3 else 0
            w.torque.y = -self.vel[4] if len(self.vel) > 4 else 0
            w.torque.z = -self.vel[5] if len(self.vel) > 5 else 0
        self.force_pub.publish(w)

    def publish_trajectory(self):
        if not self.trajectory:
            return
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        for pos in list(self.trajectory):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pos[0]
            ps.pose.position.y = pos[1]
            ps.pose.position.z = pos[2]
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.traj_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50)  # 50Hz
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now
            if dt <= 0 or dt > 0.5:
                dt = 0.02

            if self.pose is None:
                rate.sleep()
                continue

            if self.state == "TAKEOFF":
                # Auto takeoff to target altitude
                target = np.array([0.0, 0.0, self.takeoff_target])
                force = self.compute_control(target, dt)
                self.publish_force(force)

                error_z = abs(self.pose[2] - self.takeoff_target)
                vel_mag = np.linalg.norm(self.vel) if self.vel is not None else 0

                if error_z < self.pos_tolerance and vel_mag < self.vel_tolerance:
                    if self.hover_start is None:
                        self.hover_start = now
                    elif (now - self.hover_start).to_sec() > 2.0:
                        rospy.loginfo("Takeoff complete! Starting waypoint navigation.")
                        self.state = "NAVIGATE"
                        self.hover_start = None
                        self.current_wp_idx = 0
                else:
                    self.hover_start = None

                if self.pose[2] > 1.0 and not self.mission_started:
                    self.mission_started = True
                    rospy.loginfo(f"Taking off... Altitude: {self.pose[2]:.1f}m")

            elif self.state == "NAVIGATE":
                target = np.array(self.waypoints[self.current_wp_idx])
                force = self.compute_control(target, dt)
                self.publish_force(force)

                dist = np.linalg.norm(self.pose - target)
                if dist < self.pos_tolerance:
                    rospy.loginfo(f"Waypoint {self.current_wp_idx + 1} reached! "
                                  f"({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
                    self.current_wp_idx += 1
                    if self.current_wp_idx >= len(self.waypoints):
                        rospy.loginfo("All waypoints visited. Landing...")
                        self.state = "LAND"
                        self.land_start = now

                rospy.loginfo_throttle(2, f"[NAV] WP{self.current_wp_idx+1}/{len(self.waypoints)} "
                                       f"→ ({target[0]:.1f},{target[1]:.1f},{target[2]:.1f}) "
                                       f"dist={dist:.2f}m alt={self.pose[2]:.1f}m")

            elif self.state == "LAND":
                target = np.array([self.pose[0], self.pose[1], 0.3])
                force = self.compute_control(target, dt)
                force[2] = max(force[2], 0)  # don't pull down
                self.publish_force(force)
                rospy.loginfo_throttle(1, f"[LAND] Altitude: {self.pose[2]:.2f}m")

                if self.pose[2] < 0.4:
                    self.publish_force(np.zeros(3))
                    rospy.loginfo("Landing complete! Mission accomplished.")
                    self.state = "DONE"
                    break

            # Publish trajectory periodically
            self._traj_counter = getattr(self, '_traj_counter', 0) + 1
            if self._traj_counter % 25 == 0:  # ~2Hz at 50Hz loop
                self.publish_trajectory()

            rate.sleep()

        rospy.loginfo("Autonomous mission finished.")
        # Hold position
        while not rospy.is_shutdown():
            self.publish_force(np.zeros(3))
            rate.sleep()


if __name__ == '__main__':
    controller = DroneController()
    controller.run()
