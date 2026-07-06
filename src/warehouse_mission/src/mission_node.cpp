/// @file mission_node.cpp
/// MissionManager v2 — event-driven lifecycle FSM (ADR-003).
///
/// States: BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED
///
/// Transitions are triggered by EventBus events, NOT by polling.
/// Uses while(ros::ok()) + spinOnce + rate.sleep() main loop.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include "warehouse_utils/event_bus.hpp"

using warehouse_utils::EventBus;
using warehouse_utils::EventType;

class MissionManager {
public:
  MissionManager() : nh_("~") {
    map_sub_ = nh_.subscribe("/map", 1, &MissionManager::mapCb, this);
    state_pub_  = nh_.advertise<std_msgs::String>("/mission_state", 1);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("/exploration_enable", 1, true);
    cmd_pub_    = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_recovery", 1);

    // ── Subscribe to events ───────────────────────────────
    auto& bus = EventBus::instance();
    bus.subscribe(EventType::MAP_READY, [this](EventType) {
      if (state_ == WAIT_MAP) {
        transition("INITIAL_SCAN");
        state_ = INITIAL_SCAN;
        scan_start_ = ros::Time::now();
      }
    });
    bus.subscribe(EventType::EXPLORATION_DONE, [this](EventType) {
      if (state_ == EXPLORATION) {
        transition("FINISHED");
        state_ = FINISHED;
      }
    });

    transition("BOOT");
    ROS_INFO("MissionManager ready (event-driven). State: BOOT");
  }

  void spin() {
    ros::Rate rate(20);
    while (ros::ok()) {
      ros::spinOnce();
      update();
      rate.sleep();
    }
  }

private:
  enum State { BOOT, WAIT_MAP, INITIAL_SCAN, EXPLORATION, FINISHED };
  State state_ = BOOT;
  ros::Time scan_start_;
  bool has_map_ = false;

  void transition(const std::string& name) {
    std_msgs::String msg;
    msg.data = name;
    state_pub_.publish(msg);
    ROS_INFO_STREAM("Mission: " << name);
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!has_map_ && msg->info.width > 0) {
      has_map_ = true;
      EventBus::instance().emit(EventType::MAP_READY);
    }
  }

  void update() {
    double elapsed = (ros::Time::now() - scan_start_).toSec();

    switch (state_) {
      case BOOT:
        if (elapsed > 1.0 && scan_start_.isZero()) {
          // Fake: BOOT → WAIT_MAP after 1s
          // (MAP_READY event will trigger WAIT_MAP → INITIAL_SCAN)
          transition("WAIT_MAP");
          state_ = WAIT_MAP;
          scan_start_ = ros::Time::now();
        }
        break;

      case WAIT_MAP:
        // Waiting for MAP_READY event (handled in callback).
        break;

      case INITIAL_SCAN:
        if (elapsed < 8.0) {
          geometry_msgs::Twist cmd;
          cmd.angular.z = 0.8;
          cmd_pub_.publish(cmd);
        } else {
          geometry_msgs::Twist stop;
          cmd_pub_.publish(stop);
          std_msgs::Bool enable;
          enable.data = true;
          enable_pub_.publish(enable);
          EventBus::instance().emit(EventType::EXPLORATION_ENABLED);
          transition("EXPLORATION");
          state_ = EXPLORATION;
        }
        break;

      case EXPLORATION:
        // Waiting for EXPLORATION_DONE event.
        break;

      case FINISHED:
        break;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher  state_pub_, enable_pub_, cmd_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_node");
  MissionManager mgr;
  mgr.spin();
  return 0;
}
