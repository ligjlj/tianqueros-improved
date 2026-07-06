/// @file mission_node.cpp
/// Thin ROS wrapper for MissionFSM.
///
/// Subscribes: /map
/// Publishes:  /mission_state, /exploration_enable, /cmd_vel_recovery
/// FSM logic:  MissionFSM (testable in isolation)

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include "warehouse_utils/event_bus.hpp"
#include "warehouse_mission/mission_fsm.hpp"

using warehouse_utils::EventBus;
using warehouse_utils::EventType;
using warehouse_mission::MissionFSM;

class MissionNode {
public:
  MissionNode() : nh_("~") {
    map_sub_ = nh_.subscribe("/map", 1, &MissionNode::mapCb, this);

    state_pub_  = nh_.advertise<std_msgs::String>("/mission_state", 1);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("/exploration_enable", 1, true);
    cmd_pub_    = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_recovery", 1);

    // Wire FSM callbacks.
    fsm_.onStateChange([this](const std::string& name) {
      std_msgs::String msg;
      msg.data = name;
      state_pub_.publish(msg);
    });
    fsm_.onEnableChange([this](bool enable) {
      std_msgs::Bool msg;
      msg.data = enable;
      enable_pub_.publish(msg);
      if (enable) EventBus::instance().emit(EventType::EXPLORATION_ENABLED);
    });
    fsm_.onNeedRotate([this](bool active) {
      geometry_msgs::Twist cmd;
      if (active) cmd.angular.z = 0.8;
      cmd_pub_.publish(cmd);
    });

    ROS_INFO("Mission Node ready.");
  }

  void spin() {
    ros::Rate rate(20);
    while (ros::ok()) {
      ros::spinOnce();
      fsm_.update();
      rate.sleep();
    }
  }

private:
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (msg->info.width > 0) {
      fsm_.onMapReady();
      EventBus::instance().emit(EventType::MAP_READY);
      map_sub_.shutdown();  // Only need first map.
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher  state_pub_, enable_pub_, cmd_pub_;
  MissionFSM fsm_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_node");
  MissionNode node;
  node.spin();
  return 0;
}
