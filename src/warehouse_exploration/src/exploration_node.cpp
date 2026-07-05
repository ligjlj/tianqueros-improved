/// @file exploration_node.cpp
/// Thin ROS wrapper for ExplorationFSM.
///
/// Subscribes: /map, /odom, /global_path
/// Publishes:  /exploration_goal, /frontier_marker, /exploration_state
/// Recovery:   publishes /cmd_vel directly during RECOVERY/INITIAL_SCAN phase

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

#include "warehouse_utils/grid_map.hpp"
#include "warehouse_utils/csv_logger.hpp"
#include "warehouse_exploration/frontier_detector.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"
#include "warehouse_exploration/goal_selector.hpp"
#include "warehouse_exploration/exploration_fsm.hpp"

using warehouse_utils::GridMap;
using warehouse_utils::WorldPoint;
using warehouse_utils::CsvLogger;
using warehouse_exploration::ExplorationFSM;
using warehouse_exploration::FrontierDetector;
using warehouse_exploration::FrontierClusterer;

class ExplorationNode {
public:
  ExplorationNode() : nh_("~") {
    ExplorationFSM::Config fsm_cfg;
    nh_.param("goal_tolerance_m", fsm_cfg.goal_tolerance_m, 0.5);
    nh_.param("path_timeout_s",   fsm_cfg.path_timeout_s,   10.0);
    nh_.param("goal_timeout_s",   fsm_cfg.goal_timeout_s,   30.0);
    nh_.param("max_retries",      fsm_cfg.max_retries,      3);
    fsm_ = ExplorationFSM(fsm_cfg);

    GridMap::Config gcfg;
    nh_.param("inflation_radius_m", gcfg.inflation_radius_m, 1.0);
    nh_.param("robot_radius_m",     gcfg.robot_radius_m,     0.5);
    grid_ = GridMap(gcfg);

    map_sub_  = nh_.subscribe("/map", 1, &ExplorationNode::mapCb, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &ExplorationNode::odomCb, this);
    path_sub_ = nh_.subscribe("/global_path", 1, &ExplorationNode::pathCb, this);

    goal_pub_      = nh_.advertise<geometry_msgs::PoseStamped>("/exploration_goal", 1, true);
    cmd_pub_       = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    frontier_pub_  = nh_.advertise<visualization_msgs::Marker>("/frontier_marker", 1);
    state_pub_     = nh_.advertise<std_msgs::String>("/exploration_state", 1);

    csv_ = std::make_shared<CsvLogger>(
        "/tmp/exploration_log.csv",
        std::vector<std::string>{
          "state", "coverage_pct", "frontier_count",
          "goal_x", "goal_y", "robot_x", "robot_y",
          "robot_v", "retry_count", "stuck", "stagnant"});

    timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &ExplorationNode::tick, this);
    ROS_INFO("Exploration Node ready. FSM: WAIT_FOR_MAP");
  }

private:
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    try { grid_.loadFromMsg(*msg); has_map_ = true; }
    catch (const std::exception& e) {
      ROS_ERROR_STREAM("Exploration: map load failed: " << e.what());
    }
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_pos_.x = msg->pose.pose.position.x;
    robot_pos_.y = msg->pose.pose.position.y;
    robot_vx_ = msg->twist.twist.linear.x;
    robot_vy_ = msg->twist.twist.linear.y;
    robot_wz_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg->poses.empty()) {
      fsm_.onPathReceived();
    } else {
      if (fsm_.state() == ExplorationFSM::State::PLAN_PATH)
        fsm_.onPathFailed();
    }
  }

  void tick(const ros::WallTimerEvent&) {
    if (!has_map_ || !has_odom_) return;

    auto result = fsm_.tick(grid_, robot_pos_, robot_vx_, robot_vy_, robot_wz_);

    std_msgs::String state_msg;
    state_msg.data = fsm_.stateName();
    state_pub_.publish(state_msg);

    // ── Motion control (INITIAL_SCAN / RECOVERY only) ──────
    // Only this node publishes cmd_vel during scan/recovery.
    // During normal exploration, DWA controls cmd_vel.
    if (result.goal.x < -0.5) {
      geometry_msgs::Twist cmd;
      if (result.goal.x < -1.5) {
        cmd.angular.z = 0.8;   // Rotate (slower for better scan quality)
      } else {
        cmd.linear.x = -0.2;   // Backup
      }
      cmd_pub_.publish(cmd);
      was_in_motion_ = true;
    } else if (was_in_motion_) {
      // Just exited motion mode — publish stop for 1 second.
      geometry_msgs::Twist stop;
      cmd_pub_.publish(stop);
      if (++motion_stop_ticks_ > 10) {
        motion_stop_ticks_ = 0;
        was_in_motion_ = false;
      }
    }

    // During motion mode, don't publish goals.
    if (result.goal.x < -0.5 || was_in_motion_) return;

    // ── Publish exploration goal ───────────────────────────
    if (result.new_goal) {
      geometry_msgs::PoseStamped goal_msg;
      goal_msg.header.stamp    = ros::Time::now();
      goal_msg.header.frame_id = "map";
      goal_msg.pose.position.x = result.goal.x;
      goal_msg.pose.position.y = result.goal.y;
      goal_msg.pose.orientation.w = 1.0;
      goal_pub_.publish(goal_msg);
    }

    // ── Frontier visualization ─────────────────────────────
    publishFrontiers();

    // ── CSV Log ────────────────────────────────────────────
    csv_->log({
      static_cast<double>(static_cast<int>(fsm_.state())),
      result.coverage_pct,
      static_cast<double>(result.frontier_count),
      result.goal.x, result.goal.y,
      robot_pos_.x, robot_pos_.y,
      std::sqrt(robot_vx_*robot_vx_ + robot_vy_*robot_vy_),
      static_cast<double>(result.retry_count),
      fsm_.navMonitor().isStuck() ? 1.0 : 0.0,
      fsm_.mapMonitor().isStagnant() ? 1.0 : 0.0
    });
  }

  void publishFrontiers() {
    FrontierDetector det;
    auto cells = det.findFrontierCells(grid_);
    auto clusters = FrontierClusterer().cluster(cells);

    visualization_msgs::Marker marker;
    marker.header.stamp    = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns   = "frontiers";
    marker.id   = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = grid_.resolution() * 2;
    marker.scale.y = grid_.resolution() * 2;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.6;

    for (const auto& cl : clusters)
      for (const auto& cell : cl.cells) {
        auto wp = grid_.gridToWorld(cell.row, cell.col);
        geometry_msgs::Point p;
        p.x = wp.x; p.y = wp.y; p.z = 0.02;
        marker.points.push_back(p);
      }
    frontier_pub_.publish(marker);
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, odom_sub_, path_sub_;
  ros::Publisher  goal_pub_, cmd_pub_, frontier_pub_, state_pub_;
  ros::WallTimer  timer_;

  GridMap        grid_;
  ExplorationFSM fsm_;
  WorldPoint     robot_pos_{0, 0};
  double         robot_vx_ = 0.0, robot_vy_ = 0.0, robot_wz_ = 0.0;

  bool has_map_  = false;
  bool has_odom_ = false;
  bool was_in_motion_ = false;
  int  motion_stop_ticks_ = 0;

  std::shared_ptr<CsvLogger> csv_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ExplorationNode node;
  ros::spin();
  return 0;
}
