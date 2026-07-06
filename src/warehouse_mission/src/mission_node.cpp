/// @file mission_node.cpp
/// MissionManager — lifecycle FSM (ADR-003: event-driven).
///
/// States:
///   BOOT           — waiting for ROS init
///   WAIT_MAP       — waiting for /map data
///   INITIAL_SCAN   — rotate 360° to build initial map
///   EXPLORATION     — enable exploration, wait for completion
///   FINISHED        — mission complete
///
/// Publishes:
///   /mission_state       (std_msgs/String)
///   /exploration_enable  (std_msgs/Bool, latched)
///   /cmd_vel_recovery    (geometry_msgs/Twist) — only during INITIAL_SCAN
///
/// Does NOT know about Planner, Frontier, or DWA.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class MissionManager {
public:
  MissionManager() : nh_("~") {
    // Subscribers
    map_sub_ = nh_.subscribe("/map", 1, &MissionManager::mapCb, this);

    // Publishers
    state_pub_ = nh_.advertise<std_msgs::String>("/mission_state", 1);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("/exploration_enable", 1, true);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_recovery", 1);

    // Start in BOOT
    transition("BOOT");
    ROS_INFO("MissionManager ready. State: BOOT");
  }

  void spin() {
    ros::Rate rate(20);  // 20 Hz main loop
    while (ros::ok()) {
      ros::spinOnce();
      update();
      rate.sleep();
    }
  }

private:
  enum State { BOOT, WAIT_MAP, INITIAL_SCAN, EXPLORATION, FINISHED };
  State state_ = BOOT;
  ros::Time state_start_;
  bool has_map_ = false;

  void transition(const std::string& name) {
    state_start_ = ros::Time::now();
    std_msgs::String msg;
    msg.data = name;
    state_pub_.publish(msg);
    ROS_INFO_STREAM("Mission: " << name);
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    has_map_ = (msg->info.width > 0 && msg->info.height > 0);
  }

  void update() {
    double elapsed = (ros::Time::now() - state_start_).toSec();

    switch (state_) {

      case BOOT:
        // Wait for ROS to settle (1s).
        if (elapsed > 1.0) {
          state_ = WAIT_MAP;
          transition("WAIT_MAP");
        }
        break;

      case WAIT_MAP:
        if (has_map_) {
          state_ = INITIAL_SCAN;
          transition("INITIAL_SCAN");
        }
        break;

      case INITIAL_SCAN: {
        // Rotate for 8 seconds to scan surroundings.
        if (elapsed < 8.0) {
          geometry_msgs::Twist cmd;
          cmd.angular.z = 0.8;
          cmd_pub_.publish(cmd);
        } else {
          // Stop.
          geometry_msgs::Twist stop;
          cmd_pub_.publish(stop);
          // Enable exploration.
          std_msgs::Bool enable;
          enable.data = true;
          enable_pub_.publish(enable);
          state_ = EXPLORATION;
          transition("EXPLORATION");
        }
        break;
      }

      case EXPLORATION:
        // ExplorationFSM handles the rest.
        // MissionManager just waits for FINISHED signal
        // (future: subscribe to /exploration_done).
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
