#include "warehouse_exploration/goal_selector.hpp"
#include <cmath>
#include <queue>
#include <algorithm>
#include <limits>
#include <ros/console.h>

namespace warehouse_exploration {

GoalSelector::ScoredGoal GoalSelector::select(
    const std::vector<FrontierCluster>& clusters,
    const WorldPoint& robot_pos,
    const GridMap& grid) const {

  ScoredGoal best;
  best.score = -std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cl = clusters[i];
    if (cl.size < config_.min_cluster_size) continue;

    // ── Find SAFE goal cell (not mathematical center) ─────
    GridCell safe_cell = findSafeGoalCell(cl.center, grid);
    WorldPoint goal = grid.gridToWorld(safe_cell.row, safe_cell.col);

    // ── Distance score (closer = better) ──────────────────
    const double dx = goal.x - robot_pos.x;
    const double dy = goal.y - robot_pos.y;
    const double dist = std::sqrt(dx*dx + dy*dy);

    // Skip goals too close to robot (avoid trivial loop).
    if (dist < config_.min_goal_distance_m) continue;

    const double dist_score = -config_.weight_distance * dist;

    // ── Information score ─────────────────────────────────
    const double info_score = config_.weight_information
                            * computeInformation(cl, grid);

    // ── Size score ────────────────────────────────────────
    const double size_score = config_.weight_size * cl.size;

    // ── Fail penalty ──────────────────────────────────────
    auto it = fail_counts_.find(static_cast<int>(i));
    const int fails = (it != fail_counts_.end()) ? it->second : 0;
    const double fail_score = -config_.weight_fail * fails;

    const double total = dist_score + info_score + size_score + fail_score;

    if (total > best.score) {
      best.score         = total;
      best.goal          = goal;
      best.dist_score    = dist_score;
      best.info_score    = info_score;
      best.size_score    = size_score;
      best.fail_score    = fail_score;
      best.cluster_index = static_cast<int>(i);
    }
  }

  if (best.cluster_index >= 0) {
    ROS_INFO_STREAM("GoalSelector: cluster #" << best.cluster_index
                    << " score=" << best.score
                    << " d=" << best.dist_score
                    << " i=" << best.info_score
                    << " s=" << best.size_score
                    << " f=" << best.fail_score);
  }

  return best;
}

GridCell GoalSelector::findSafeGoalCell(
    const GridCell& center, const GridMap& grid) const {

  // If center itself is safe, use it.
  if (grid.isValid(center.row, center.col) &&
      grid.isFree(center.row, center.col) &&
      grid.isPassable(center.row, center.col)) {
    return center;
  }

  // BFS in expanding radius to find nearest free+safe cell.
  const int R = config_.safe_placement_radius;
  GridCell best = center;
  double best_dist = std::numeric_limits<double>::max();

  for (int dr = -R; dr <= R; ++dr) {
    for (int dc = -R; dc <= R; ++dc) {
      const int nr = center.row + dr;
      const int nc = center.col + dc;
      if (!grid.isValid(nr, nc)) continue;
      if (!grid.isFree(nr, nc)) continue;
      if (!grid.isPassable(nr, nc)) continue;  // Must survive inflation

      const double d2 = dr*dr + dc*dc;
      if (d2 < best_dist) {
        best_dist = d2;
        best = GridCell{nr, nc};
      }
    }
  }

  return best;
}

double GoalSelector::computeInformation(
    const FrontierCluster& cluster,
    const GridMap& grid) const {

  static constexpr int DR[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int DC[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  int unknown_count = 0;
  for (const auto& cell : cluster.cells) {
    for (int k = 0; k < 8; ++k) {
      const int nr = cell.row + DR[k];
      const int nc = cell.col + DC[k];
      if (grid.isValid(nr, nc) && grid.isUnknown(nr, nc))
        ++unknown_count;
    }
  }
  return cluster.size > 0
    ? static_cast<double>(unknown_count) / cluster.size : 0.0;
}

}  // namespace warehouse_exploration
