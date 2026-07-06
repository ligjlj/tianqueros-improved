#ifndef WAREHOUSE_EXPLORATION_COVERAGE_MONITOR_HPP_
#define WAREHOUSE_EXPLORATION_COVERAGE_MONITOR_HPP_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <mutex>

namespace warehouse_exploration {

/// Background coverage monitor (ADR-004: Worker pattern).
/// Runs at 2 Hz, independent of FSM. Subscribes to /map,
/// computes coverage %, publishes /coverage.
class CoverageMonitor {
public:
  CoverageMonitor() {
    nh_ = ros::NodeHandle("~");
    map_sub_ = nh_.subscribe("/map", 1, &CoverageMonitor::mapCb, this);
    pub_ = nh_.advertise<std_msgs::Float32>("/coverage", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(0.5), &CoverageMonitor::tick, this);
    ROS_INFO("CoverageMonitor ready (2 Hz background).");
  }

  void spin() { ros::spin(); }

private:
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_ = *msg;
    has_map_ = true;
  }

  void tick(const ros::WallTimerEvent&) {
    if (!has_map_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    int known = 0, total = latest_.info.width * latest_.info.height;
    if (total == 0) return;
    for (int i = 0; i < total; ++i)
      if (latest_.data[i] >= 0) ++known;
    std_msgs::Float32 msg;
    msg.data = 100.0f * known / total;
    pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher  pub_;
  ros::WallTimer  timer_;
  nav_msgs::OccupancyGrid latest_;
  bool has_map_ = false;
  std::mutex mtx_;
};

}  // namespace warehouse_exploration
#endif
