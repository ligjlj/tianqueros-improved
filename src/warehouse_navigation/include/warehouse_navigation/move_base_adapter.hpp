#ifndef WAREHOUSE_NAVIGATION_MOVE_BASE_ADAPTER_HPP_
#define WAREHOUSE_NAVIGATION_MOVE_BASE_ADAPTER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace warehouse_navigation {

/// Adapter wrapping move_base action client.
/// Usage: adapter.sendGoal(pose) → SUCCEEDED/ABORTED/PENDING
class MoveBaseAdapter {
public:
  enum State { IDLE, ACTIVE, SUCCEEDED, ABORTED };

  MoveBaseAdapter(const std::string& name = "move_base")
    : ac_(name, true) {
    ROS_INFO("MoveBaseAdapter: waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("MoveBaseAdapter: connected.");
  }

  void sendGoal(const geometry_msgs::PoseStamped& pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    state_ = ACTIVE;
    ac_.sendGoal(goal,
      boost::bind(&MoveBaseAdapter::doneCb, this, _1, _2),
      boost::bind(&MoveBaseAdapter::activeCb, this),
      boost::bind(&MoveBaseAdapter::feedbackCb, this, _1));
  }

  void cancel() { ac_.cancelAllGoals(); state_ = IDLE; }
  State state() const { return state_; }
  bool isDone() const { return state_ == SUCCEEDED || state_ == ABORTED; }

private:
  void doneCb(const actionlib::SimpleClientGoalState& st,
              const move_base_msgs::MoveBaseResultConstPtr&) {
    state_ = (st == actionlib::SimpleClientGoalState::SUCCEEDED)
             ? SUCCEEDED : ABORTED;
  }
  void activeCb() { state_ = ACTIVE; }
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr&) {}

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  State state_ = IDLE;
};

}  // namespace warehouse_navigation
#endif
