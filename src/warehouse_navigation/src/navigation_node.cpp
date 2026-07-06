/// NavigationManager (PR7): monitors navigation, publishes /nav_status + /nav_stuck.
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

class NavigationNode {
  ros::NodeHandle nh_;
  ros::Subscriber path_sub_;
  ros::Publisher  status_pub_, stuck_pub_;
  ros::WallTimer  timer_;
  bool has_path_ = false;
  ros::WallTime last_path_time_;

public:
  NavigationNode() {
    path_sub_ = nh_.subscribe("/global_path", 1, &NavigationNode::pathCb, this);
    status_pub_ = nh_.advertise<std_msgs::String>("/nav_status", 1);
    stuck_pub_  = nh_.advertise<std_msgs::Bool>("/nav_stuck", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &NavigationNode::tick, this);
    ROS_INFO("NavigationManager ready.");
  }

  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    has_path_ = !msg->poses.empty();
    if (has_path_) last_path_time_ = ros::WallTime::now();
  }

  void tick(const ros::WallTimerEvent&) {
    std_msgs::String s;
    if (!has_path_) s.data = "IDLE";
    else if ((ros::WallTime::now() - last_path_time_).toSec() > 10.0) {
      s.data = "STUCK";
      std_msgs::Bool st; st.data = true; stuck_pub_.publish(st);
    } else s.data = "FOLLOWING";
    status_pub_.publish(s);
  }
};

int main(int argc, char** argv) { ros::init(argc,argv,"navigation_node"); NavigationNode n; ros::spin(); return 0; }
