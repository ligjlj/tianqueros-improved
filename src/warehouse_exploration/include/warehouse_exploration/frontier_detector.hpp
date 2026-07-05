#ifndef WAREHOUSE_EXPLORATION_FRONTIER_DETECTOR_HPP_
#define WAREHOUSE_EXPLORATION_FRONTIER_DETECTOR_HPP_

#include <vector>
#include "warehouse_utils/grid_map.hpp"

namespace warehouse_exploration {

/// Frontier cell detector.
///
/// Scans the occupancy grid for frontier cells: free cells adjacent to
/// at least one unknown cell. This is step 1 of the exploration pipeline.
///
/// Does NOT cluster, score, or select goals — those are separate classes.
class FrontierDetector {
public:
  struct Config {
    int min_frontier_size = 1;  ///< Minimum cluster size (used by caller)
  };

  explicit FrontierDetector(const Config& cfg) : config_(cfg) {}
  FrontierDetector() = default;

  /// Find all frontier cells (free cells adjacent to at least one unknown).
  std::vector<warehouse_utils::GridCell> findFrontierCells(
      const warehouse_utils::GridMap& grid) const;

  const Config& config() const { return config_; }

private:
  Config config_;
};

}  // namespace warehouse_exploration

#endif
