#include "warehouse_controller/dwa_planner.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <ros/console.h>

namespace warehouse_controller {

using warehouse_utils::GridMap;
using warehouse_utils::WorldPoint;

DWAPlanner::DWAPlanner(const Config& cfg) : config_(cfg) {}

// ═══════════════════════════════════════════════════════════════
// Main entry point
// ═══════════════════════════════════════════════════════════════

std::pair<double, double> DWAPlanner::computeVelocity(
    const GridMap& grid,
    const RobotState& state,
    const WorldPoint& goal) const {

  // ── Dynamic window ──────────────────────────────────────
  double v_min, v_max, w_min, w_max;
  computeDynamicWindow(state, v_min, v_max, w_min, w_max);

  // ── Sample velocities ──────────────────────────────────
  Trajectory best;
  best.score = -std::numeric_limits<double>::infinity();
  best.v = 0.0;
  best.w = 0.0;

  const double dv = (config_.v_samples > 1)
    ? (v_max - v_min) / (config_.v_samples - 1) : 0.0;
  const double dw = (config_.w_samples > 1)
    ? (w_max - w_min) / (config_.w_samples - 1) : 0.0;

  for (int iv = 0; iv < config_.v_samples; ++iv) {
    const double v = v_min + iv * dv;
    for (int iw = 0; iw < config_.w_samples; ++iw) {
      const double w = w_min + iw * dw;
      auto traj = simulate(grid, state, v, w, goal);
      if (traj.score > best.score) {
        best = traj;
      }
    }
  }

  ROS_DEBUG_STREAM("DWA: best (v=" << best.v << ", w=" << best.w
                   << ") score=" << best.score
                   << " h=" << best.heading_score
                   << " c=" << best.clearance_score
                   << " s=" << best.speed_score);

  return {best.v, best.w};
}

// ═══════════════════════════════════════════════════════════════
// All trajectories (for visualization)
// ═══════════════════════════════════════════════════════════════

std::vector<DWAPlanner::Trajectory> DWAPlanner::getTrajectories(
    const GridMap& grid,
    const RobotState& state,
    const WorldPoint& goal) const {

  double v_min, v_max, w_min, w_max;
  computeDynamicWindow(state, v_min, v_max, w_min, w_max);

  const double dv = (config_.v_samples > 1)
    ? (v_max - v_min) / (config_.v_samples - 1) : 0.0;
  const double dw = (config_.w_samples > 1)
    ? (w_max - w_min) / (config_.w_samples - 1) : 0.0;

  std::vector<Trajectory> results;
  for (int iv = 0; iv < config_.v_samples; ++iv) {
    const double v = v_min + iv * dv;
    for (int iw = 0; iw < config_.w_samples; ++iw) {
      const double w = w_min + iw * dw;
      results.push_back(simulate(grid, state, v, w, goal));
    }
  }
  return results;
}

// ═══════════════════════════════════════════════════════════════
// Trajectory simulation
// ═══════════════════════════════════════════════════════════════

DWAPlanner::Trajectory DWAPlanner::simulate(
    const GridMap& grid,
    const RobotState& state,
    double v, double w,
    const WorldPoint& goal) const {

  Trajectory traj;
  traj.v = v;
  traj.w = w;
  traj.collided = false;

  double x = state.x;
  double y = state.y;
  double theta = state.theta;
  const int n_steps = static_cast<int>(config_.predict_time / config_.dt);

  double min_clearance = std::numeric_limits<double>::infinity();

  for (int step = 0; step < n_steps; ++step) {
    x     += v * std::cos(theta) * config_.dt;
    y     += v * std::sin(theta) * config_.dt;
    theta += w * config_.dt;

    // Normalize theta.
    theta = std::atan2(std::sin(theta), std::cos(theta));

    traj.points.push_back({x, y});

    // Collision check using inflated grid.
    auto cell = grid.worldToGrid(x, y);
    if (!grid.isValid(cell.row, cell.col)) {
      traj.collided = true;
      break;
    }
    const double cost = grid.getInflatedCost(cell.row, cell.col);
    if (cost > 253.0) {
      traj.collided = true;
      break;
    }

    // Track minimum clearance (for scoring).
    if (cost < min_clearance) {
      min_clearance = cost;
    }
  }

  if (traj.collided) {
    traj.score           = -std::numeric_limits<double>::infinity();
    traj.heading_score   = 0.0;
    traj.clearance_score = 0.0;
    traj.speed_score     = 0.0;
    return traj;
  }

  // ── Heading score ───────────────────────────────────────
  // How well the final heading aligns with the goal direction.
  const double goal_angle = std::atan2(goal.y - y, goal.x - x);
  double heading_error = goal_angle - theta;
  heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
  traj.heading_score = (M_PI - std::abs(heading_error)) / M_PI;

  // ── Clearance score ─────────────────────────────────────
  // Normalised: 0 (touching obstacle) to 1 (free).
  // min_clearance is the inflated cost (0=free, 254=lethal).
  traj.clearance_score = 1.0 - std::min(min_clearance / 254.0, 1.0);

  // ── Speed score ─────────────────────────────────────────
  traj.speed_score = (config_.max_linear_vel > 0.0)
    ? v / config_.max_linear_vel : 0.0;

  // ── Combined ────────────────────────────────────────────
  traj.score = config_.alpha * traj.heading_score
             + config_.beta  * traj.clearance_score
             + config_.gamma * traj.speed_score;

  return traj;
}

// ═══════════════════════════════════════════════════════════════
// Dynamic window
// ═══════════════════════════════════════════════════════════════

void DWAPlanner::computeDynamicWindow(
    const RobotState& state,
    double& v_min, double& v_max,
    double& w_min, double& w_max) const {

  // Absolute limits.
  v_min = 0.0;   // DWA doesn't usually allow reverse.
  v_max = config_.max_linear_vel;
  w_min = -config_.max_angular_vel;
  w_max =  config_.max_angular_vel;

  // Acceleration limits (given dt).
  const double a_v = config_.max_linear_accel  * config_.dt;
  const double a_w = config_.max_angular_accel * config_.dt;

  v_min = std::max(v_min, state.v - a_v);
  v_max = std::min(v_max, state.v + a_v);
  w_min = std::max(w_min, state.w - a_w);
  w_max = std::min(w_max, state.w + a_w);
}

// ═══════════════════════════════════════════════════════════════
// Goal check
// ═══════════════════════════════════════════════════════════════

bool DWAPlanner::isGoalReached(const RobotState& state,
                               const WorldPoint& goal) const {
  const double dx = goal.x - state.x;
  const double dy = goal.y - state.y;
  return std::sqrt(dx * dx + dy * dy) < config_.goal_tolerance_m;
}

}  // namespace warehouse_controller
