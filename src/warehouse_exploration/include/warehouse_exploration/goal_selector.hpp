#ifndef WAREHOUSE_EXPLORATION_GOAL_SELECTOR_HPP_
#define WAREHOUSE_EXPLORATION_GOAL_SELECTOR_HPP_

#include <vector>
#include <unordered_map>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"

namespace warehouse_exploration {

using warehouse_utils::WorldPoint;
using warehouse_utils::GridMap;
using warehouse_utils::GridCell;

/// Weighted goal selector with safe placement and fail-count penalty.
class GoalSelector {
public:
  struct Config {
    double weight_distance     = 0.3;
    double weight_information  = 2.0;
    double weight_size         = 1.5;
    double weight_fail         = 5.0;   ///< Penalty per previous failure
    int    min_cluster_size    = 5;
    double min_goal_distance_m = 2.0;   ///< Skip goals closer than this
    int    safe_placement_radius = 10;
  };

  struct ScoredGoal {
    WorldPoint goal;
    double     score       = 0.0;
    double     dist_score  = 0.0;
    double     info_score  = 0.0;
    double     size_score  = 0.0;
    double     fail_score  = 0.0;
    int        cluster_index = -1;
  };

  explicit GoalSelector(const Config& cfg) : config_(cfg) {}
  GoalSelector() = default;

  /// Set fail counts for goal positions (from GoalManager).
  void setFailCounts(const std::unordered_map<int, int>& fail_map) {
    fail_counts_ = fail_map;
  }

  /// Score all clusters and return the best goal.
  ScoredGoal select(
      const std::vector<FrontierCluster>& clusters,
      const WorldPoint& robot_pos,
      const GridMap& grid) const;

  const Config& config() const { return config_; }

private:
  /// Find the nearest free + inflated-safe cell to a given grid position.
  GridCell findSafeGoalCell(const GridCell& center, const GridMap& grid) const;

  /// Count unknown neighbors for a cluster.
  double computeInformation(const FrontierCluster& cluster,
                            const GridMap& grid) const;

  Config config_;
  mutable std::unordered_map<int, int> fail_counts_;  ///< cluster_index → fail_count
};

}  // namespace warehouse_exploration

#endif
