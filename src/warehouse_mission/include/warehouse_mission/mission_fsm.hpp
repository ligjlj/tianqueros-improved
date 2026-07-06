#ifndef WAREHOUSE_MISSION_MISSION_FSM_HPP_
#define WAREHOUSE_MISSION_MISSION_FSM_HPP_

#include <string>
#include <functional>
#include <ros/time.h>
#include <ros/duration.h>

namespace warehouse_mission {

/// Mission lifecycle FSM (ADR-003: testable in isolation).
///
/// States: BOOT → WAIT_MAP → INITIAL_SCAN → EXPLORATION → FINISHED
///
/// This class is ROS-agnostic. Dependencies are injected via callbacks:
///   - publish_state(name): called on every state transition
///   - publish_enable(bool): called when entering/leaving EXPLORATION
///   - need_rotate(bool): true=start rotating, false=stop
class MissionFSM {
public:
  enum State { BOOT, WAIT_MAP, INITIAL_SCAN, EXPLORATION, FINISHED };

  using StateCallback = std::function<void(const std::string&)>;
  using BoolCallback  = std::function<void(bool)>;

  MissionFSM() = default;

  /// Inject dependencies.
  void onStateChange(StateCallback cb)  { state_cb_ = std::move(cb); }
  void onEnableChange(BoolCallback cb)  { enable_cb_ = std::move(cb); }
  void onNeedRotate(BoolCallback cb)    { rotate_cb_ = std::move(cb); }

  /// Tick the FSM. Call at 20 Hz.
  void update();

  /// External events.
  void onMapReady();
  void onExplorationDone();
  void onYawUpdate(double current_yaw);  ///< Feed current odom yaw for INITIAL_SCAN verification.

  // Accessors.
  State state() const { return state_; }
  const std::string& stateName() const;
  double timeInState() const { return (ros::WallTime::now() - state_start_).toSec(); }

  /// Reset for testing.
  void reset() { state_ = BOOT; state_start_ = ros::WallTime::now(); }

private:
  void transition(State s);

  State state_      = BOOT;
  ros::WallTime state_start_ = ros::WallTime::now();

  StateCallback state_cb_;
  BoolCallback  enable_cb_;
  BoolCallback  rotate_cb_;

  bool map_ready_          = false;
  bool exploration_done_   = false;
  ros::WallTime scan_start_{0, 0};
  double last_scan_yaw_    = 0.0;    ///< Previous odom yaw for delta integration
  double scan_accum_yaw_   = 0.0;    ///< Accumulated absolute yaw change during INITIAL_SCAN
  bool   scan_yaw_valid_   = false;  ///< True once first odom yaw received
};

}  // namespace warehouse_mission

#endif
