#ifndef WAREHOUSE_CONTROLLER_DWA_PLANNER_HPP_
#define WAREHOUSE_CONTROLLER_DWA_PLANNER_HPP_

#include <vector>
#include <utility>
#include "warehouse_utils/grid_map.hpp"

namespace warehouse_controller {

/// DWA (Dynamic Window Approach) local planner.
///
/// Samples linear and angular velocities within a dynamic window
/// (bounded by current velocity ± acceleration limits and absolute max),
/// forward-simulates trajectories, and scores them on three criteria:
///   1. Heading — alignment with the goal direction
///   2. Clearance — distance to the nearest obstacle (via inflated cost)
///   3. Speed — prefers higher linear velocity
///
/// Outputs a single (v, w) pair to be published as cmd_vel.
class DWAPlanner {
public:
  /// A single velocity sample (v, w) and its trajectory.
  struct Trajectory {
    double v;                       ///< Linear velocity (m/s)
    double w;                       ///< Angular velocity (rad/s)
    double score;                   ///< Combined score (higher is better)
    double heading_score;
    double clearance_score;
    double speed_score;
    bool   collided;                ///< True if trajectory hits an obstacle
    std::vector<warehouse_utils::WorldPoint> points;  ///< Simulated poses
  };

  /// Current robot state.
  struct RobotState {
    double x;       ///< World x (m)
    double y;       ///< World y (m)
    double theta;   ///< Heading (rad)
    double v;       ///< Current linear velocity (m/s)
    double w;       ///< Current angular velocity (rad/s)
  };

  /// Configuration.
  struct Config {
    double max_linear_vel    = 1.0;
    double max_angular_vel   = 1.5;
    double max_linear_accel  = 2.0;
    double max_angular_accel = 3.0;
    double predict_time      = 1.5;    // seconds
    double dt                = 0.1;    // seconds
    int    v_samples         = 10;
    int    w_samples         = 20;
    double alpha             = 0.5;   // heading weight
    double beta              = 2.0;   // clearance weight
    double gamma             = 0.3;   // speed weight
    double goal_tolerance_m  = 0.3;
  };

  // ── Construction ────────────────────────────────────────
  explicit DWAPlanner(const Config& cfg);
  DWAPlanner() = default;

  // ── Planning ────────────────────────────────────────────
  /// Compute the best (v, w) command given the current state,
  /// grid map, and goal position.
  /// Returns (v, w). If no safe trajectory exists, returns (0, 0).
  std::pair<double, double> computeVelocity(
      const warehouse_utils::GridMap& grid,
      const RobotState& state,
      const warehouse_utils::WorldPoint& goal) const;

  /// Get all sampled trajectories for visualization.
  std::vector<Trajectory> getTrajectories(
      const warehouse_utils::GridMap& grid,
      const RobotState& state,
      const warehouse_utils::WorldPoint& goal) const;

  // ── Goal reached check ─────────────────────────────────
  bool isGoalReached(const RobotState& state,
                     const warehouse_utils::WorldPoint& goal) const;

  const Config& config() const { return config_; }

private:
  /// Simulate a trajectory forward using constant (v, w).
  /// Returns the list of poses and whether a collision occurred.
  Trajectory simulate(const warehouse_utils::GridMap& grid,
                      const RobotState& state,
                      double v, double w,
                      const warehouse_utils::WorldPoint& goal) const;

  /// Compute the dynamic window [v_min, v_max] × [w_min, w_max]
  /// given the current state and acceleration limits.
  void computeDynamicWindow(const RobotState& state,
                            double& v_min, double& v_max,
                            double& w_min, double& w_max) const;

  Config config_;
};

}  // namespace warehouse_controller

#endif  // WAREHOUSE_CONTROLLER_DWA_PLANNER_HPP_
