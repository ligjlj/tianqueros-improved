#ifndef WAREHOUSE_EXPLORATION_FRONTIER_CLUSTER_HPP_
#define WAREHOUSE_EXPLORATION_FRONTIER_CLUSTER_HPP_

#include <vector>
#include <queue>
#include <unordered_set>
#include "warehouse_utils/grid_map.hpp"

namespace warehouse_exploration {

using warehouse_utils::GridCell;

/// A cluster of connected frontier cells.
struct FrontierCluster {
  std::vector<GridCell> cells;
  GridCell center{0, 0};     ///< Centroid in grid coords
  int    size = 0;            ///< Number of cells
  int    min_row = 0, max_row = 0;  ///< Bounding box
  int    min_col = 0, max_col = 0;
};

/// BFS-based connected-components clustering for frontier cells.
///
/// Pure algorithm class — no ROS dependencies.
class FrontierClusterer {
public:
  /// Cluster frontier cells into connected components (8-connectivity).
  /// Returns empty vector if no cells.
  std::vector<FrontierCluster> cluster(
      const std::vector<GridCell>& frontier_cells) const;
};

// ── Hash for unordered_set ──────────────────────────────────
struct CellHash {
  std::size_t operator()(const GridCell& c) const {
    return static_cast<std::size_t>(c.row) * 65537 +
           static_cast<std::size_t>(c.col);
  }
};

}  // namespace warehouse_exploration

#endif
