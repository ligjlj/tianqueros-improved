#ifndef WAREHOUSE_EXPLORATION_REACHABILITY_CHECKER_HPP_
#define WAREHOUSE_EXPLORATION_REACHABILITY_CHECKER_HPP_

#include <vector>
#include <queue>
#include "warehouse_utils/grid_map.hpp"

namespace warehouse_exploration {

using warehouse_utils::GridMap;
using warehouse_utils::GridCell;

/// BFS-based reachability checker.
///
/// Flood-fills from the robot position and marks which grid cells are connected.
/// Then checks each cluster center for connectivity.
/// O(width × height) — much cheaper than running A* for every cluster.
class ReachabilityChecker {
public:
  /// Filter clusters: keep only those whose center is reachable from robot_pos.
  /// Returns indices of reachable clusters in the original vector.
  std::vector<int> filterReachable(
      const GridMap& grid,
      const std::vector<GridCell>& robot_positions,
      const std::vector<std::vector<GridCell>>& cluster_cells) const;

  /// Simple check: is there a free path from start to goal via BFS?
  /// Uses 8-connectivity and inflated cost for collision checking.
  bool isReachable(const GridMap& grid,
                   const GridCell& start,
                   const GridCell& goal) const;
};

}  // namespace warehouse_exploration

#endif
