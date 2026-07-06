#include "warehouse_exploration/goal_manager.hpp"
#include <cmath>
#include <ros/console.h>

namespace warehouse_exploration {

int GoalManager::FindRecord(const WorldPoint& g) {
  for (size_t i = 0; i < records_.size(); ++i) {
    double dx = records_[i].goal.x - g.x, dy = records_[i].goal.y - g.y;
    if (std::sqrt(dx*dx + dy*dy) < kDedupRadius) return static_cast<int>(i);
  }
  return -1;
}

int GoalManager::FindRecord(const WorldPoint& g) const {
  for (size_t i = 0; i < records_.size(); ++i) {
    double dx = records_[i].goal.x - g.x, dy = records_[i].goal.y - g.y;
    if (std::sqrt(dx*dx + dy*dy) < kDedupRadius) return static_cast<int>(i);
  }
  return -1;
}

void GoalManager::addCandidate(const WorldPoint& g) {
  if (FindRecord(g) >= 0) return;
  records_.push_back({g, GoalState::UNTRIED, 0, ros::WallTime(0), ros::WallTime(0)});
}

void GoalManager::markTried(const WorldPoint& g) {
  int i = FindRecord(g);
  if (i < 0) { addCandidate(g); i = static_cast<int>(records_.size())-1; }
  records_[i].state = GoalState::TRIED;
  records_[i].last_attempt = ros::WallTime::now();
}

void GoalManager::markFailed(const WorldPoint& g) {
  int i = FindRecord(g);
  if (i < 0) { addCandidate(g); i = static_cast<int>(records_.size())-1; }
  auto& r = records_[i];
  r.fail_count++;
  r.last_attempt = ros::WallTime::now();
  if (r.fail_count >= kBlacklistThreshold) {
    r.state = GoalState::BLACKLISTED;
    r.blacklist_until = ros::WallTime::now() + ros::WallDuration(60.0);
    ROS_WARN_STREAM("Goal blacklisted (" << r.fail_count << " failures)");
  } else {
    r.state = GoalState::FAILED;
  }
}

void GoalManager::markCompleted(const WorldPoint& g) {
  int i = FindRecord(g);
  if (i < 0) return;
  records_[i].fail_count = 0;
  records_[i].state = GoalState::UNTRIED;
}

bool GoalManager::isBlacklisted(const WorldPoint& g, double cooldown_s) {
  int i = FindRecord(g);
  if (i < 0) return false;
  auto& r = records_[i];
  if (r.state == GoalState::BLACKLISTED && ros::WallTime::now() > r.blacklist_until) {
    r.state = GoalState::UNTRIED; r.fail_count = 0; return false;
  }
  return r.state == GoalState::BLACKLISTED;
}

int GoalManager::getFailCount(const WorldPoint& g) const {
  int i = FindRecord(g);
  return (i >= 0) ? records_[i].fail_count : 0;
}

GoalState GoalManager::getState(const WorldPoint& g) const {
  int i = FindRecord(g);
  return (i >= 0) ? records_[i].state : GoalState::UNTRIED;
}

std::unordered_map<int, int> GoalManager::getFailMap(
    const std::vector<FrontierCluster>& clusters,
    const GridMap& grid) const {
  std::unordered_map<int, int> m;
  for (size_t i = 0; i < clusters.size(); ++i) {
    WorldPoint g = grid.gridToWorld(clusters[i].center.row, clusters[i].center.col);
    int fc = getFailCount(g);
    if (fc > 0) m[static_cast<int>(i)] = fc;
  }
  return m;
}

void GoalManager::clear() { records_.clear(); }

}  // namespace warehouse_exploration
