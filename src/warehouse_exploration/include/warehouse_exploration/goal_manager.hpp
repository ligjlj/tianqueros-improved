#ifndef WAREHOUSE_EXPLORATION_GOAL_MANAGER_HPP_
#define WAREHOUSE_EXPLORATION_GOAL_MANAGER_HPP_

#include <vector>
#include <unordered_map>
#include <ros/time.h>
#include <ros/duration.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"

namespace warehouse_exploration {

using warehouse_utils::WorldPoint;
using warehouse_utils::GridMap;

/// Goal state machine for exploration targets.
enum class GoalState { UNTRIED, TRIED, FAILED, BLACKLISTED };

/// Manages exploration goal lifecycle (ADR-006: Blackboard-like).
///
/// Tracks each goal through states: UNTRIED → TRIED → FAILED → BLACKLISTED.
/// Blacklisted goals auto-expire after cooldown.
/// Deduplicates goals within kDedupRadius.
class GoalManager {
public:
  struct GoalRecord {
    WorldPoint goal;
    GoalState  state = GoalState::UNTRIED;
    int        fail_count = 0;
    ros::WallTime blacklist_until;
    ros::WallTime last_attempt;
  };

  void addCandidate(const WorldPoint& g);
  void markTried(const WorldPoint& g);
  void markFailed(const WorldPoint& g);
  void markCompleted(const WorldPoint& g);

  bool isBlacklisted(const WorldPoint& g, double cooldown_s = 60.0);
  int  getFailCount(const WorldPoint& g) const;
  GoalState getState(const WorldPoint& g) const;

  std::unordered_map<int, int> getFailMap(
      const std::vector<FrontierCluster>& clusters,
      const GridMap& grid) const;

  void clear();
  size_t size() const { return records_.size(); }

private:
  static constexpr double kDedupRadius = 0.5;
  static constexpr int    kBlacklistThreshold = 3;
  std::vector<GoalRecord> records_;
  int FindRecord(const WorldPoint& g);
  int FindRecord(const WorldPoint& g) const;
};

}  // namespace warehouse_exploration
#endif
