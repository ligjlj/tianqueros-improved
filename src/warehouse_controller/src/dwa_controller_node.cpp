/// @file dwa_controller_node.cpp
/// ROS node wrapper for DWAPlanner.
///
/// Subscribes:
///   /map          — nav_msgs::OccupancyGrid
///   /global_path  — nav_msgs::Path (from A* planner)
///   /odom         — nav_msgs::Odometry (robot state: pose + velocity)
///
/// Publishes:
///   /cmd_vel      — geometry_msgs::Twist
///   /local_path   — nav_msgs::Path (best trajectory for viz)
///   /dwa_traj     — visualization_msgs::Marker (all sampled trajectories)

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include "warehouse_utils/grid_map.hpp"
#include "warehouse_controller/dwa_planner.hpp"

class DWAControllerNode {
public:
  DWAControllerNode() : nh_("~") {
    // ── Load DWA config ───────────────────────────────────
    warehouse_controller::DWAPlanner::Config cfg;
    nh_.param("max_linear_vel",    cfg.max_linear_vel,    1.0);
    nh_.param("max_angular_vel",   cfg.max_angular_vel,   1.5);
    nh_.param("max_linear_accel",  cfg.max_linear_accel,  2.0);
    nh_.param("max_angular_accel", cfg.max_angular_accel, 3.0);
    nh_.param("predict_time",      cfg.predict_time,      1.5);
    nh_.param("dt",                cfg.dt,                0.1);
    nh_.param("v_samples",         cfg.v_samples,         10);
    nh_.param("w_samples",         cfg.w_samples,         20);
    nh_.param("alpha",             cfg.alpha,             0.5);
    nh_.param("beta",              cfg.beta,              2.0);
    nh_.param("gamma",             cfg.gamma,             0.3);
    nh_.param("goal_tolerance_m",  cfg.goal_tolerance_m,  0.3);

    // GridMap config.
    warehouse_utils::GridMap::Config gcfg;
    nh_.param("inflation_radius_m", gcfg.inflation_radius_m, 1.0);
    nh_.param("robot_radius_m",     gcfg.robot_radius_m,     0.5);

    grid_    = warehouse_utils::GridMap(gcfg);
    planner_ = warehouse_controller::DWAPlanner(cfg);

    // ── Subscribers ───────────────────────────────────────
    map_sub_  = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "/map", 1, &DWAControllerNode::mapCallback, this);
    path_sub_ = nh_.subscribe<nav_msgs::Path>(
        "/global_path", 1, &DWAControllerNode::pathCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/odom", 1, &DWAControllerNode::odomCallback, this);

    // ── Publishers ────────────────────────────────────────
    cmd_pub_       = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    traj_pub_       = nh_.advertise<visualization_msgs::Marker>("/dwa_traj", 1);

    // ── Control loop timer (10 Hz) ────────────────────────
    timer_ = nh_.createTimer(ros::Duration(0.1),
                             &DWAControllerNode::controlLoop, this);

    ROS_INFO("DWA Controller Node ready.");
  }

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    try { grid_.loadFromMsg(*msg); has_map_ = true; }
    catch (const std::exception& e) {
      ROS_ERROR_STREAM("DWA: failed to load map: " << e.what());
    }
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    global_path_ = msg;
    if (!msg->poses.empty()) {
      const auto& p = msg->poses.back().pose.position;
      goal_ = warehouse_utils::WorldPoint{p.x, p.y};
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    state_.x     = msg->pose.pose.position.x;
    state_.y     = msg->pose.pose.position.y;

    // Extract yaw from quaternion.
    const auto& q = msg->pose.pose.orientation;
    state_.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    state_.v = msg->twist.twist.linear.x;
    state_.w = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!has_map_ || !has_odom_ || !global_path_) return;

    // Check goal reached.
    if (planner_.isGoalReached(state_, goal_)) {
      geometry_msgs::Twist stop;
      cmd_pub_.publish(stop);
      ROS_INFO_THROTTLE(5, "DWA: goal reached.");
      return;
    }

    // Compute best velocity.
    auto [v, w] = planner_.computeVelocity(grid_, state_, goal_);

    // Publish cmd_vel.
    geometry_msgs::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = w;
    cmd_pub_.publish(cmd);

    // Publish best local path.
    publishLocalPath(v, w);

    // Publish all trajectories (throttled to 2 Hz to save bandwidth).
    if (traj_count_++ % 5 == 0) {
      publishTrajectories();
    }

    ROS_DEBUG_STREAM("DWA: v=" << v << " w=" << w
                     << " pos=(" << state_.x << "," << state_.y << ")");
  }

  void publishLocalPath(double v, double w) {
    nav_msgs::Path lp;
    lp.header.stamp    = ros::Time::now();
    lp.header.frame_id = "map";

    double x = state_.x, y = state_.y, th = state_.theta;
    const int n = static_cast<int>(config().predict_time / config().dt);

    for (int i = 0; i < n; ++i) {
      x  += v * std::cos(th) * config().dt;
      y  += v * std::sin(th) * config().dt;
      th += w * config().dt;

      geometry_msgs::PoseStamped pose;
      pose.header = lp.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      lp.poses.push_back(pose);
    }
    local_path_pub_.publish(lp);
  }

  void publishTrajectories() {
    auto trajs = planner_.getTrajectories(grid_, state_, goal_);

    visualization_msgs::Marker marker;
    marker.header.stamp    = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns   = "dwa_traj";
    marker.id   = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.3;

    for (const auto& traj : trajs) {
      for (size_t i = 1; i < traj.points.size(); ++i) {
        geometry_msgs::Point p1, p2;
        p1.x = traj.points[i-1].x; p1.y = traj.points[i-1].y; p1.z = 0.05;
        p2.x = traj.points[i].x;   p2.y = traj.points[i].y;   p2.z = 0.05;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }
    traj_pub_.publish(marker);
  }

  const warehouse_controller::DWAPlanner::Config& config() const {
    return planner_.config();
  }

  // ── ROS ─────────────────────────────────────────────────
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, path_sub_, odom_sub_;
  ros::Publisher  cmd_pub_, local_path_pub_, traj_pub_;
  ros::Timer      timer_;

  // ── Core ────────────────────────────────────────────────
  warehouse_utils::GridMap          grid_;
  warehouse_controller::DWAPlanner  planner_;

  // ── State ───────────────────────────────────────────────
  warehouse_controller::DWAPlanner::RobotState state_;
  warehouse_utils::WorldPoint goal_{0, 0};
  nav_msgs::Path::ConstPtr   global_path_;
  bool has_map_  = false;
  bool has_odom_ = false;
  int  traj_count_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dwa_controller_node");
  DWAControllerNode node;
  ros::spin();
  return 0;
}
