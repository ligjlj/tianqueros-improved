#include "warehouse_planner/astar_planner.hpp"

#include <chrono>
#include <algorithm>
#include <ros/console.h>

namespace warehouse_planner {

using warehouse_utils::GridMap;
using warehouse_utils::GridCell;
using warehouse_utils::WorldPoint;

// ═══════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════

AStarPlanner::AStarPlanner(const Config& cfg)
  : config_(cfg)
{}

// ═══════════════════════════════════════════════════════════════
// Planning — public interface
// ═══════════════════════════════════════════════════════════════

AStarPlanner::Result AStarPlanner::plan(
    const GridMap& grid,
    const WorldPoint& start,
    const WorldPoint& goal) const {

  return planGrid(grid,
                  grid.worldToGrid(start.x, start.y),
                  grid.worldToGrid(goal.x, goal.y));
}

AStarPlanner::Result AStarPlanner::planGrid(
    const GridMap& grid,
    const GridCell& start_cell,
    const GridCell& goal_cell) const {

  Result result;
  const auto t_start = std::chrono::steady_clock::now();

  // ── Pre-checks ──────────────────────────────────────────
  if (!grid.isValid(start_cell.row, start_cell.col)) {
    ROS_WARN_STREAM("A*: start cell (" << start_cell.row << "," << start_cell.col
                    << ") is outside map.");
    return result;
  }
  if (!grid.isValid(goal_cell.row, goal_cell.col)) {
    ROS_WARN_STREAM("A*: goal cell (" << goal_cell.row << "," << goal_cell.col
                    << ") is outside map.");
    return result;
  }
  if (!isTraversable(grid, start_cell.row, start_cell.col)) {
    ROS_WARN_STREAM("A*: start cell is not traversable.");
    return result;
  }
  if (!isTraversable(grid, goal_cell.row, goal_cell.col)) {
    ROS_WARN_STREAM("A*: goal cell is not traversable.");
    return result;
  }

  // ── A* loop ─────────────────────────────────────────────
  std::priority_queue<OpenNode> open_set;
  std::unordered_map<GridCell, GridCell, CellHash, CellEqual> came_from;
  std::unordered_map<GridCell, double, CellHash, CellEqual> g_score;

  const double h_start = heuristic(start_cell, goal_cell);
  open_set.push({h_start, 0.0, start_cell.row, start_cell.col});
  g_score[start_cell] = 0.0;

  // 8-connected neighbors: {dr, dc, cost}
  static constexpr int    N_DIRS = 8;
  static constexpr int    DR[N_DIRS] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int    DC[N_DIRS] = {-1,  0,  1, -1, 1, -1, 0, 1};
  static constexpr double COST[N_DIRS] = {
    1.414, 1.0, 1.414,
    1.0,         1.0,
    1.414, 1.0, 1.414
  };

  while (!open_set.empty()) {
    const OpenNode current = open_set.top();
    open_set.pop();

    const GridCell cur_cell{current.row, current.col};

    // Skip stale entries (node already expanded with lower g_score).
    auto it_g = g_score.find(cur_cell);
    if (it_g != g_score.end() && current.g_score > it_g->second + 1e-9) {
      continue;
    }

    ++result.nodes_expanded;

    // Goal reached.
    if (cur_cell.row == goal_cell.row && cur_cell.col == goal_cell.col) {
      std::vector<GridCell> grid_path;
      reconstructPath(came_from, cur_cell, grid_path);

      // Convert grid path → world path.
      for (const auto& gc : grid_path) {
        result.path.push_back(grid.gridToWorld(gc.row, gc.col));
      }

      // Compute path length.
      for (size_t i = 1; i < result.path.size(); ++i) {
        const double dx = result.path[i].x - result.path[i - 1].x;
        const double dy = result.path[i].y - result.path[i - 1].y;
        result.path_length_m += std::sqrt(dx * dx + dy * dy);
      }

      result.success = true;
      goto done;
    }

    // Expand neighbors.
    for (int k = 0; k < N_DIRS; ++k) {
      const int nr = current.row + DR[k];
      const int nc = current.col + DC[k];

      if (!grid.isValid(nr, nc))  continue;
      if (!isTraversable(grid, nr, nc)) continue;

      const double tentative_g = current.g_score + COST[k] * grid.resolution();
      const GridCell neighbor{nr, nc};

      auto it = g_score.find(neighbor);
      if (it != g_score.end() && tentative_g >= it->second - 1e-9) {
        continue;  // Already reached with better or equal cost.
      }

      came_from[neighbor] = cur_cell;
      g_score[neighbor]    = tentative_g;
      const double h        = heuristic(neighbor, goal_cell);
      const double f        = tentative_g + h;
      open_set.push({f, tentative_g, nr, nc});
    }
  }

  // No path found.

done:
  const auto t_end = std::chrono::steady_clock::now();
  result.planning_time_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (result.success) {
    ROS_INFO_STREAM("A*: path found! " << result.path.size() << " waypoints, "
                    << result.path_length_m << " m, "
                    << result.nodes_expanded << " nodes, "
                    << result.planning_time_ms << " ms");
  } else {
    ROS_WARN_STREAM("A*: no path found. Expanded "
                    << result.nodes_expanded << " nodes in "
                    << result.planning_time_ms << " ms");
  }

  return result;
}

// ═══════════════════════════════════════════════════════════════
// Heuristic
// ═══════════════════════════════════════════════════════════════

double AStarPlanner::heuristic(const GridCell& a, const GridCell& b) const {
  const double dr = static_cast<double>(a.row - b.row);
  const double dc = static_cast<double>(a.col - b.col);
  return config_.heuristic_weight * std::sqrt(dr * dr + dc * dc);
}

// ═══════════════════════════════════════════════════════════════
// Traversability
// ═══════════════════════════════════════════════════════════════

bool AStarPlanner::isTraversable(const GridMap& grid,
                                 int row, int col) const {
  // Use the inflated cost with a slightly higher threshold
  // (collision_margin_m adds safety). The inflated grid already
  // has costs 0–254, where 254 = lethal. We treat anything above
  // a margin-adjusted threshold as blocked.
  constexpr double kLethalBase = 253.0;
  const double margin_cells = config_.collision_margin_m / grid.resolution();
  // Simple approach: if cost > 253 - margin, it's blocked.
  const double threshold = std::max(0.0, kLethalBase - margin_cells * 10.0);
  return grid.getInflatedCost(row, col) < threshold;
}

// ═══════════════════════════════════════════════════════════════
// Path Reconstruction
// ═══════════════════════════════════════════════════════════════

void AStarPlanner::reconstructPath(
    const std::unordered_map<GridCell, GridCell, CellHash, CellEqual>& came_from,
    GridCell current,
    std::vector<GridCell>& out) const {

  out.clear();
  out.push_back(current);

  auto it = came_from.find(current);
  while (it != came_from.end()) {
    current = it->second;
    out.push_back(current);
    it = came_from.find(current);
  }

  std::reverse(out.begin(), out.end());
}

}  // namespace warehouse_planner
