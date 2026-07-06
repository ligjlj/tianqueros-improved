#include "warehouse_mission/mission_fsm.hpp"
#include <ros/console.h>

namespace warehouse_mission {

static const char* kNames[] = {"BOOT","WAIT_MAP","INITIAL_SCAN","EXPLORATION","FINISHED"};

const std::string& MissionFSM::stateName() const {
  static std::string s;
  s = kNames[static_cast<int>(state_)];
  return s;
}

void MissionFSM::transition(State s) {
  state_ = s;
  state_start_ = ros::WallTime::now();
  if (state_cb_) state_cb_(stateName());
  ROS_INFO_STREAM("MissionFSM: → " << kNames[static_cast<int>(s)]);
}

void MissionFSM::onMapReady() { map_ready_ = true; }
void MissionFSM::onExplorationDone() { exploration_done_ = true; }

void MissionFSM::update() {
  double elapsed = timeInState();

  switch (state_) {
    case BOOT:
      if (elapsed > 1.0) transition(WAIT_MAP);
      break;

    case WAIT_MAP:
      if (map_ready_) {
        map_ready_ = false;
        transition(INITIAL_SCAN);
        scan_start_ = ros::WallTime::now();
        if (rotate_cb_) rotate_cb_(true);
      }
      break;

    case INITIAL_SCAN:
      if ((ros::WallTime::now() - scan_start_).toSec() > 8.0) {
        if (rotate_cb_) rotate_cb_(false);
        if (enable_cb_) enable_cb_(true);
        transition(EXPLORATION);
      }
      break;

    case EXPLORATION:
      if (exploration_done_) {
        exploration_done_ = false;
        if (enable_cb_) enable_cb_(false);
        transition(FINISHED);
      }
      break;

    case FINISHED:
      break;
  }
}

}  // namespace warehouse_mission
