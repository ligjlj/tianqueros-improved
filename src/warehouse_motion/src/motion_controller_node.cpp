/// @file motion_controller_node.cpp
/// MotionController — sole /cmd_vel publisher (ADR-001).
///
/// Priority-based multiplexer:
///   Priority 3: RECOVERY / INITIAL_SCAN (safety-critical)
///   Priority 2: DWA local planner (navigation)
///   Priority 1: Default (idle → stop)
///
/// Subscribes to one input topic per priority level.
/// Publishes the highest-priority non-zero command.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>

class MotionController {
public:
  MotionController() : nh_("~") {
    // ── Priority-ordered subscribers ──────────────────────
    // Each input comes from a different module.
    // The raw cmd_vel from legacy nodes is remapped in launch.
    sub_recovery_ = nh_.subscribe("/cmd_vel_recovery", 1,
        &MotionController::recoveryCb, this);
    sub_nav_      = nh_.subscribe("/cmd_vel_nav", 1,
        &MotionController::navCb, this);

    // ── Sole output ───────────────────────────────────────
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // ── Timer: publish highest-priority command at 20 Hz ──
    timer_ = nh_.createTimer(ros::Duration(0.05),
        &MotionController::publishLoop, this);

    ROS_INFO("MotionController ready. /cmd_vel is the sole output.");
  }

private:
  void recoveryCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_recovery_ = *msg;
    has_recovery_ = true;
    recovery_timeout_ = ros::Time::now() + ros::Duration(0.5);
  }

  void navCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_nav_ = *msg;
    has_nav_ = true;
    nav_timeout_ = ros::Time::now() + ros::Duration(0.5);
  }

  void publishLoop(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;

    // Priority 3: Recovery (timeout after 0.5s of no messages).
    if (has_recovery_ && ros::Time::now() < recovery_timeout_) {
      cmd = last_recovery_;
    }
    // Priority 2: Navigation (DWA).
    else if (has_nav_ && ros::Time::now() < nav_timeout_) {
      cmd = last_nav_;
    }
    // Priority 1: Stop (default).

    pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_recovery_, sub_nav_;
  ros::Publisher  pub_;
  ros::Timer      timer_;

  geometry_msgs::Twist last_recovery_, last_nav_;
  bool    has_recovery_ = false, has_nav_ = false;
  ros::Time recovery_timeout_, nav_timeout_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_controller");
  MotionController mc;
  ros::spin();
  return 0;
}
