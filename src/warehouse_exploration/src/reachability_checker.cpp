#include "warehouse_exploration/reachability_checker.hpp"
#include <unordered_set>
#include <ros/console.h>

namespace warehouse_exploration {

// Hash for GridCell
struct RCellHash {
  std::size_t operator()(const GridCell& c) const {
    return static_cast<std::size_t>(c.row) * 65537 +
           static_cast<std::size_t>(c.col);
  }
};

std::vector<int> ReachabilityChecker::filterReachable(
    const GridMap& grid,
    const std::vector<GridCell>& robot_positions,
    const std::vector<std::vector<GridCell>>& cluster_cells) const {

  if (robot_positions.empty() || cluster_cells.empty()) return {};

  // BFS from robot position through all passable cells.
  std::unordered_set<GridCell, RCellHash> visited;
  std::queue<GridCell> q;

  for (const auto& start : robot_positions) {
    if (grid.isValid(start.row, start.col) && grid.isFree(start.row, start.col)) {
      q.push(start);
      visited.insert(start);
    }
  }

  static constexpr int DR[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int DC[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  while (!q.empty()) {
    GridCell cur = q.front(); q.pop();
    for (int k = 0; k < 8; ++k) {
      GridCell nb{cur.row + DR[k], cur.col + DC[k]};
      if (!grid.isValid(nb.row, nb.col)) continue;
      if (!grid.isFree(nb.row, nb.col)) continue;
      if (visited.count(nb)) continue;
      visited.insert(nb);
      q.push(nb);
    }
  }

  // Check each cluster: is at least one cell in the cluster reachable?
  std::vector<int> reachable_indices;
  for (size_t i = 0; i < cluster_cells.size(); ++i) {
    for (const auto& cell : cluster_cells[i]) {
      if (visited.count(cell)) {
        reachable_indices.push_back(static_cast<int>(i));
        break;
      }
    }
  }

  ROS_DEBUG_STREAM("Reachability: " << reachable_indices.size()
                   << "/" << cluster_cells.size() << " clusters reachable");
  return reachable_indices;
}

bool ReachabilityChecker::isReachable(
    const GridMap& grid,
    const GridCell& start,
    const GridCell& goal) const {

  if (!grid.isValid(start.row, start.col) || !grid.isFree(start.row, start.col))
    return false;
  if (!grid.isValid(goal.row, goal.col) || !grid.isFree(goal.row, goal.col))
    return false;

  std::unordered_set<GridCell, RCellHash> visited;
  std::queue<GridCell> q;
  q.push(start);
  visited.insert(start);

  static constexpr int DR[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int DC[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  while (!q.empty()) {
    GridCell cur = q.front(); q.pop();
    if (cur.row == goal.row && cur.col == goal.col) return true;

    for (int k = 0; k < 8; ++k) {
      GridCell nb{cur.row + DR[k], cur.col + DC[k]};
      if (!grid.isValid(nb.row, nb.col)) continue;
      if (!grid.isFree(nb.row, nb.col)) continue;  // Raw free check, no inflation
      if (visited.count(nb)) continue;
      visited.insert(nb);
      q.push(nb);
    }
  }
  return false;
}

}  // namespace warehouse_exploration
