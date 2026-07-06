/// NavigationManager v2 — uses ROS Navigation Stack via MoveBaseAdapter.
/// Subscribes to /exploration_goal → sends to move_base → publishes /nav_status.
/// Dual-track: config nav_planner = "move_base" or "self"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "warehouse_navigation/move_base_adapter.hpp"

using warehouse_navigation::MoveBaseAdapter;

class NavigationNode {
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Publisher  status_pub_;
  MoveBaseAdapter adapter_;
  ros::WallTimer  timer_;

public:
  NavigationNode() : adapter_("move_base") {
    goal_sub_ = nh_.subscribe("/exploration_goal", 1, &NavigationNode::goalCb, this);
    status_pub_ = nh_.advertise<std_msgs::String>("/nav_status", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &NavigationNode::tick, this);
    ROS_INFO("NavigationManager ready (move_base adapter).");
  }

  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& g) {
    ROS_INFO_STREAM("Nav goal: (" << g->pose.position.x << ", " << g->pose.position.y << ")");
    adapter_.sendGoal(*g);
  }

  void tick(const ros::WallTimerEvent&) {
    std_msgs::String s;
    switch (adapter_.state()) {
      case MoveBaseAdapter::ACTIVE:    s.data = "FOLLOWING"; break;
      case MoveBaseAdapter::SUCCEEDED: s.data = "GOAL_REACHED"; break;
      case MoveBaseAdapter::ABORTED:   s.data = "STUCK"; break;
      default:                         s.data = "IDLE"; break;
    }
    status_pub_.publish(s);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_node");
  NavigationNode n;
  ros::spin();
  return 0;
}
