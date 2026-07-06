#ifndef WAREHOUSE_EXPLORATION_EXPLORATION_FSM_HPP_
#define WAREHOUSE_EXPLORATION_EXPLORATION_FSM_HPP_

#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <ros/time.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/frontier_detector.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"
#include "warehouse_exploration/goal_selector.hpp"
#include "warehouse_exploration/reachability_checker.hpp"
#include "warehouse_exploration/goal_manager.hpp"

namespace warehouse_exploration {

using warehouse_utils::GridMap;
using warehouse_utils::WorldPoint;
using warehouse_utils::GridCell;

// ═══════════════════════════════════════════════════════════════
// Navigation Monitor — stuck, oscillation, off-path detection
// ═══════════════════════════════════════════════════════════════
class NavigationMonitor {
public:
  struct Config {
    double stuck_velocity_threshold = 0.05;
    double stuck_duration_s         = 10.0;
    double oscillation_window_s     = 8.0;   ///< Time window for oscillation check
    int    oscillation_min_flips    = 4;     ///< Min direction changes to flag oscillation
    double path_deviation_threshold = 1.0;
  };

  /// Record current velocity command.
  void update(double vx, double vy, double wz);

  bool isStuck()       const { return stuck_; }
  bool isOscillating() const { return oscillating_; }
  bool isOffPath()     const { return off_path_; }
  void reset();

private:
  Config      config_;
  ros::WallTime stuck_start_{0, 0};
  bool    stuck_       = false;
  bool    oscillating_ = false;
  bool    off_path_    = false;

  // Oscillation detection: track angular velocity sign flips.
  struct VelSample {
    ros::WallTime t;
    double    wz;
  };
  std::deque<VelSample> vel_history_;
};

// ═══════════════════════════════════════════════════════════════
// Map Progress Monitor
// ═══════════════════════════════════════════════════════════════
class MapProgressMonitor {
public:
  struct Config {
    double stagnation_coverage_delta = 0.1;
    double stagnation_timeout_s      = 30.0;
  };
  void update(double coverage_pct);
  bool isStagnant() const { return stagnant_; }
  void reset();
private:
  Config config_;
  double last_coverage_ = 0.0;
  ros::WallTime last_progress_time_{0, 0};
  bool stagnant_ = false;
};

// ═══════════════════════════════════════════════════════════════
// Exploration FSM
// ═══════════════════════════════════════════════════════════════
class ExplorationFSM {
public:
  enum class State {
    DETECT_FRONTIER, SELECT_GOAL, PLAN_PATH, FOLLOW_PATH,
    GOAL_REACHED, RECOVERY, REPLAN, UPDATE, FINISHED
  };

  struct Config {
    double goal_tolerance_m  = 0.5;
    double path_timeout_s    = 10.0;
    double goal_timeout_s    = 30.0;
    int    max_retries       = 3;
    double blacklist_duration_s = 60.0;
  };

  ExplorationFSM() = default;
  explicit ExplorationFSM(const Config& cfg);

  struct TickResult {
    WorldPoint goal;
    bool  new_goal     = false;
    State state        = State::DETECT_FRONTIER;
    double coverage_pct = 0.0;
    int    frontier_count = 0;
    int    retry_count   = 0;
    // Recovery signals: goal.x < -0.5 = backup, goal.x < -1.5 = rotate
  };

  TickResult tick(const GridMap& grid, const WorldPoint& robot_pos,
                  double robot_vx, double robot_vy, double robot_wz);

  void onPathReceived();
  void onPathFailed();
  State state() const { return state_; }
  const std::string& stateName() const;
  const NavigationMonitor&  navMonitor()  const { return nav_monitor_; }
  const MapProgressMonitor& mapMonitor()  const { return map_monitor_; }

private:
  void transitionTo(State s);
  void handleDetectFrontier(TickResult& r, const GridMap& grid,
                            const WorldPoint& robot_pos);
  void handleSelectGoal(TickResult& r, const GridMap& grid,
                        const WorldPoint& robot_pos);
  void handlePlanPath(TickResult& r);
  void handleFollowPath(TickResult& r, const WorldPoint& robot_pos,
                        double vx, double vy, double wz);
  void handleRecovery(TickResult& r);

  Config config_;
  State  state_ = State::DETECT_FRONTIER;
  ros::WallTime state_enter_time_{0, 0};
  ros::WallTime state_enter_wall_{0, 0};
  ros::WallTime goal_start_time_{0, 0};
  int     retry_count_       = 0;
  int     coverage_tick_     = 0;
  double  cached_coverage_   = 0.0;

  WorldPoint current_goal_{0, 0};
  std::vector<FrontierCluster> last_clusters_;
  std::vector<int>              reachable_indices_;
  GoalSelector::ScoredGoal      last_scored_;

  enum class RecoveryPhase { BACKUP, ROTATE, DONE };
  RecoveryPhase recovery_phase_ = RecoveryPhase::DONE;
  ros::WallTime  recovery_phase_start_{0, 0};

  NavigationMonitor  nav_monitor_;
  GoalManager        goal_manager_;
  MapProgressMonitor map_monitor_;
  ReachabilityChecker reach_checker_;
};

}  // namespace warehouse_exploration

#endif
