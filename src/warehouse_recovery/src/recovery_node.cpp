/// RecoveryManager (PR8): listens for /nav_stuck, executes recovery via MotionController.
/// Publishes backup+rotate commands to /cmd_vel_recovery.

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class RecoveryNode {
  ros::NodeHandle nh_;
  ros::Subscriber stuck_sub_;
  ros::Publisher  cmd_pub_;
  ros::WallTimer  timer_;
  bool recovering_ = false;
  ros::WallTime recovery_start_;
  enum Phase { BACKUP, ROTATE, DONE } phase_ = DONE;

public:
  RecoveryNode() {
    stuck_sub_ = nh_.subscribe("/nav_stuck", 1, &RecoveryNode::stuckCb, this);
    cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_recovery", 1);
    timer_     = nh_.createWallTimer(ros::WallDuration(0.1), &RecoveryNode::tick, this);
    ROS_INFO("RecoveryManager ready.");
  }

  void stuckCb(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !recovering_) {
      recovering_ = true; phase_ = BACKUP; recovery_start_ = ros::WallTime::now();
      ROS_WARN("Recovery triggered: BACKUP");
    }
  }

  void tick(const ros::WallTimerEvent&) {
    if (!recovering_) return;
    double e = (ros::WallTime::now() - recovery_start_).toSec();
    geometry_msgs::Twist cmd;

    if (phase_ == BACKUP && e > 2.0) { phase_ = ROTATE; recovery_start_ = ros::WallTime::now(); ROS_INFO("Recovery: ROTATE"); }
    if (phase_ == ROTATE && e > 3.0) { phase_ = DONE; recovering_ = false; ROS_INFO("Recovery: DONE"); return; }

    if (phase_ == BACKUP) cmd.linear.x = -0.2;
    else if (phase_ == ROTATE) cmd.angular.z = 0.8;
    cmd_pub_.publish(cmd);
  }
};

int main(int argc, char** argv) { ros::init(argc,argv,"recovery_node"); RecoveryNode r; ros::spin(); return 0; }
