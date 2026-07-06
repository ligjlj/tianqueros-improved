/// MotionController — sole /cmd_vel publisher (ADR-001).
/// Priority: recovery(3) > nav(2) > stop(1).
/// Uses WallTimer/WallTime — decoupled from sim_time.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class MotionController {
public:
  MotionController() : nh_("~") {
    sub_recovery_ = nh_.subscribe("/cmd_vel_recovery", 1, &MotionController::recoveryCb, this);
    sub_nav_      = nh_.subscribe("/cmd_vel_nav", 1, &MotionController::navCb, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(0.05), &MotionController::publishLoop, this);
    ROS_INFO("MotionController ready (WallTimer). /cmd_vel sole output.");
  }

private:
  void recoveryCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_recovery_ = *msg; has_recovery_ = true;
    recovery_timeout_ = ros::WallTime::now() + ros::WallDuration(0.5);
  }
  void navCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_nav_ = *msg; has_nav_ = true;
    nav_timeout_ = ros::WallTime::now() + ros::WallDuration(0.5);
  }
  void publishLoop(const ros::WallTimerEvent&) {
    geometry_msgs::Twist cmd;
    if (has_recovery_ && ros::WallTime::now() < recovery_timeout_) cmd = last_recovery_;
    else if (has_nav_ && ros::WallTime::now() < nav_timeout_) cmd = last_nav_;
    pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_recovery_, sub_nav_;
  ros::Publisher  pub_;
  ros::WallTimer  timer_;
  geometry_msgs::Twist last_recovery_, last_nav_;
  bool has_recovery_=false, has_nav_=false;
  ros::WallTime recovery_timeout_, nav_timeout_;
};

int main(int argc, char** argv) { ros::init(argc,argv,"motion_controller"); MotionController mc; ros::spin(); return 0; }
