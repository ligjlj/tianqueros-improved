#include "warehouse_mission/mission_fsm.hpp"
#include <ros/console.h>
#include <cmath>

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

void MissionFSM::onYawUpdate(double current_yaw) {
  if (state_ != INITIAL_SCAN) return;
  // On first yaw sample, just store it as reference.
  if (!scan_yaw_valid_) {
    last_scan_yaw_ = current_yaw;
    scan_yaw_valid_ = true;
    return;
  }
  // Accumulate absolute yaw change by integrating consecutive deltas.
  double delta = current_yaw - last_scan_yaw_;
  // Normalize to [-π, π] to handle ±2π wrapping.
  while (delta >  M_PI) delta -= 2.0 * M_PI;
  while (delta < -M_PI) delta += 2.0 * M_PI;
  scan_accum_yaw_ += std::abs(delta);
  last_scan_yaw_ = current_yaw;
}

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
        scan_yaw_valid_ = false;   // Reset yaw tracking for fresh scan
        scan_accum_yaw_ = 0.0;
        if (rotate_cb_) rotate_cb_(true);
      }
      break;

    case INITIAL_SCAN:
      // Must complete full 360° rotation + minimum 8s time.
      // WallTime elapsed + actual odom yaw accumulated.
      // Safety: also allow timeout at 14s even if yaw tracking failed.
      {
        bool timed_out = (ros::WallTime::now() - scan_start_).toSec() > 14.0;
        bool scanned_enough = scan_yaw_valid_ && scan_accum_yaw_ >= 5.5  // ~315°
                           && (ros::WallTime::now() - scan_start_).toSec() > 8.0;
        if (timed_out || scanned_enough) {
          if (rotate_cb_) rotate_cb_(false);
          if (enable_cb_) enable_cb_(true);
          transition(EXPLORATION);
        }
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
