#include "warehouse_exploration/exploration_fsm.hpp"
#include "warehouse_exploration/goal_manager.hpp"
#include <ros/console.h>
#include <cmath>

namespace warehouse_exploration {

// ═══════════════════════════════════════════════════════════════
// Navigation Monitor
// ═══════════════════════════════════════════════════════════════

void NavigationMonitor::update(double vx, double vy, double wz) {
  const double speed = std::sqrt(vx*vx + vy*vy);
  const auto now = ros::WallTime::now();

  // Stuck detection.
  if (speed < config_.stuck_velocity_threshold) {
    if (stuck_start_.isZero()) stuck_start_ = now;
    else if ((now - stuck_start_).toSec() > config_.stuck_duration_s)
      stuck_ = true;
  } else {
    stuck_start_ = ros::WallTime(0, 0);
    stuck_ = false;
  }

  // Oscillation detection: count angular velocity sign flips.
  vel_history_.push_back({now, wz});
  // Purge old samples.
  while (!vel_history_.empty() &&
         (now - vel_history_.front().t).toSec() > config_.oscillation_window_s)
    vel_history_.pop_front();

  int flips = 0;
  if (vel_history_.size() >= 2) {
    double prev = vel_history_.front().wz;
    for (size_t i = 1; i < vel_history_.size(); ++i) {
      double cur = vel_history_[i].wz;
      if ((prev > 0.05 && cur < -0.05) || (prev < -0.05 && cur > 0.05)) {
        ++flips;
      }
      prev = cur;
    }
  }
  oscillating_ = (flips >= config_.oscillation_min_flips);
}

void NavigationMonitor::reset() {
  stuck_start_ = ros::WallTime(0, 0);
  stuck_ = false;
  oscillating_ = false;
  off_path_ = false;
  vel_history_.clear();
}

// ═══════════════════════════════════════════════════════════════
// FSM
// ═══════════════════════════════════════════════════════════════

void MapProgressMonitor::update(double c) {
  if (c > last_coverage_ + config_.stagnation_coverage_delta) {
    last_coverage_ = c;
    last_progress_time_ = ros::WallTime::now();
    stagnant_ = false;
  } else if (!last_progress_time_.isZero() &&
             (ros::WallTime::now() - last_progress_time_).toSec()
              > config_.stagnation_timeout_s) {
    stagnant_ = true;
  }
  if (last_progress_time_.isZero()) {
    last_progress_time_ = ros::WallTime::now();
    last_coverage_ = c;
  }
}

void MapProgressMonitor::reset() {
  last_coverage_ = 0.0;
  last_progress_time_ = ros::WallTime::now();
  stagnant_ = false;
}

// ═══════════════════════════════════════════════════════════════
// FSM
// ═══════════════════════════════════════════════════════════════

static const char* kStateNames[] = {
  "DETECT_FRONTIER","SELECT_GOAL","PLAN_PATH","FOLLOW_PATH",
  "GOAL_REACHED","RECOVERY","REPLAN","UPDATE","FINISHED"
};

ExplorationFSM::ExplorationFSM(const Config& cfg) : config_(cfg) {}

const std::string& ExplorationFSM::stateName() const {
  static std::string s;
  s = kStateNames[static_cast<int>(state_)];
  return s;
}

void ExplorationFSM::transitionTo(State s) {
  ROS_INFO_STREAM("FSM: " << kStateNames[static_cast<int>(state_)]
                  << " → " << kStateNames[static_cast<int>(s)]);
  state_ = s;
  state_enter_time_ = ros::WallTime::now();
  state_enter_wall_ = ros::WallTime::now();
}

ExplorationFSM::TickResult ExplorationFSM::tick(
    const GridMap& grid, const WorldPoint& robot_pos,
    double vx, double vy, double wz) {

  TickResult r;
  r.state = state_;
  // Coverage: only compute every 2 seconds (expensive O(N²) scan).
  int known = 0;
  if (coverage_tick_++ % 20 == 0) {  // Every 20 ticks @ 10Hz = 2s
    for (int ri = 0; ri < grid.height(); ++ri)
      for (int c = 0; c < grid.width(); ++c)
        if (!grid.isUnknown(ri, c)) ++known;
    cached_coverage_ = (grid.width() * grid.height() > 0)
      ? 100.0 * known / (grid.width() * grid.height()) : 0.0;
  }
  r.coverage_pct = cached_coverage_;

  map_monitor_.update(r.coverage_pct);

  switch (state_) {
    case State::DETECT_FRONTIER: handleDetectFrontier(r, grid, robot_pos); break;
    case State::SELECT_GOAL:     handleSelectGoal(r, grid, robot_pos); break;
    case State::PLAN_PATH:       handlePlanPath(r); break;
    case State::FOLLOW_PATH:     handleFollowPath(r, robot_pos, vx, vy, wz); break;
    case State::RECOVERY:        handleRecovery(r); break;
    case State::REPLAN:          transitionTo(State::SELECT_GOAL); break;
    case State::GOAL_REACHED:
      goal_manager_.markCompleted(current_goal_);
      transitionTo(State::UPDATE); break;
    case State::UPDATE:          transitionTo(State::DETECT_FRONTIER); break;
    case State::FINISHED:
      ROS_INFO_THROTTLE(10, "FINISHED. %.1f%%", r.coverage_pct); break;
    default: break;
  }

  r.state = state_;
  return r;
}

// ── DETECT_FRONTIER + REACHABILITY FILTER ─────────────────────
void ExplorationFSM::handleDetectFrontier(TickResult& r,
    const GridMap& grid, const WorldPoint& robot_pos) {

  FrontierDetector det;
  auto cells = det.findFrontierCells(grid);

  if (cells.empty()) {
    // Don't finish immediately — wait for map to grow.
    // Only finish if we have meaningful coverage and still no frontiers.
    if (r.coverage_pct < 2.0) {
      // Map barely started — keep waiting.
      ROS_INFO_THROTTLE(3, "Waiting for map to grow... (%.1f%%)", r.coverage_pct);
    } else if (map_monitor_.isStagnant()) {
      ROS_INFO("No frontiers + map stagnant. FINISHED.");
      transitionTo(State::FINISHED);
    }
    return;
  }

  last_clusters_ = FrontierClusterer().cluster(cells);

  // All clusters are candidates — A* will verify actual reachability.
  reachable_indices_.clear();
  for (size_t i = 0; i < last_clusters_.size(); ++i)
    reachable_indices_.push_back(static_cast<int>(i));

  r.frontier_count = static_cast<int>(reachable_indices_.size());
  ROS_INFO_STREAM("Frontiers: " << last_clusters_.size() << " total");

  transitionTo(State::SELECT_GOAL);
}

// ── SELECT_GOAL ───────────────────────────────────────────────
void ExplorationFSM::handleSelectGoal(TickResult& r,
    const GridMap& grid, const WorldPoint& robot_pos) {

  // Build filtered cluster list (only reachable, not blacklisted).
  std::vector<FrontierCluster> candidates;
  std::vector<int> orig_indices;
  for (int idx : reachable_indices_) {
    WorldPoint g = grid.gridToWorld(
        last_clusters_[idx].center.row, last_clusters_[idx].center.col);
    if (!goal_manager_.isBlacklisted(g, config_.blacklist_duration_s)) {
      candidates.push_back(last_clusters_[idx]);
      orig_indices.push_back(idx);
    }
  }

  if (candidates.empty()) {
    ROS_WARN("All reachable frontiers blacklisted. Clearing blacklist...");
    goal_manager_.clear();
    candidates = last_clusters_;
    for (size_t i = 0; i < candidates.size(); ++i) orig_indices.push_back(i);
  }

  // Score with fail-count penalty.
  GoalSelector::Config gcfg;
  GoalSelector selector(gcfg);
  selector.setFailCounts(goal_manager_.getFailMap(candidates, grid));

  last_scored_ = selector.select(candidates, robot_pos, grid);

  if (last_scored_.cluster_index < 0) {
    transitionTo(State::FINISHED);
    return;
  }

  // Map back to original cluster index.
  int real_idx = orig_indices[last_scored_.cluster_index];
  current_goal_ = last_scored_.goal;
  goal_manager_.addCandidate(current_goal_);
  retry_count_ = 0;
  nav_monitor_.reset();

  r.goal     = current_goal_;
  r.new_goal = true;
  transitionTo(State::PLAN_PATH);
}

// ── PLAN_PATH ─────────────────────────────────────────────────
void ExplorationFSM::handlePlanPath(TickResult& r) {
  r.goal = current_goal_;
  double elapsed = (ros::WallTime::now() - state_enter_wall_).toSec();
  if (elapsed > config_.path_timeout_s) {
    ROS_WARN_STREAM("Path timeout (" << elapsed << "s).");
    onPathFailed();
  }
}

// ── FOLLOW_PATH ───────────────────────────────────────────────
void ExplorationFSM::handleFollowPath(TickResult& r,
    const WorldPoint& robot_pos, double vx, double vy, double wz) {

  r.goal = current_goal_;
  nav_monitor_.update(vx, vy, wz);

  const double dx = robot_pos.x - current_goal_.x;
  const double dy = robot_pos.y - current_goal_.y;
  const double dist = std::sqrt(dx*dx + dy*dy);

  // Goal reached.
  if (dist < config_.goal_tolerance_m) {
    ROS_INFO("Goal reached.");
    retry_count_ = 0;
    transitionTo(State::GOAL_REACHED);
    return;
  }

  // Stuck or oscillating → recovery.
  if (nav_monitor_.isStuck() || nav_monitor_.isOscillating()) {
    ROS_WARN("Robot stuck/oscillating. Recovery.");
    transitionTo(State::RECOVERY);
    return;
  }

  // Timeout.
  double elapsed = (ros::WallTime::now() - goal_start_time_).toSec();
  if (elapsed > config_.goal_timeout_s) {
    retry_count_++;
    if (retry_count_ >= config_.max_retries) {
      goal_manager_.markFailed(current_goal_);
      retry_count_ = 0;
      transitionTo(State::SELECT_GOAL);
    } else {
      transitionTo(State::REPLAN);
    }
    return;
  }

  // Map stagnation during movement.
  if (map_monitor_.isStagnant() && elapsed > 20.0) {
    ROS_WARN("Map not growing. Switching goal.");
    transitionTo(State::SELECT_GOAL);
  }
}

// ── RECOVERY ──────────────────────────────────────────────────
void ExplorationFSM::handleRecovery(TickResult& r) {
  if (recovery_phase_ == RecoveryPhase::DONE) {
    recovery_phase_ = RecoveryPhase::BACKUP;
    recovery_phase_start_ = ros::WallTime::now();
    ROS_INFO("Recovery: BACKUP 2s...");
  }

  double elapsed = (ros::WallTime::now() - recovery_phase_start_).toSec();

  switch (recovery_phase_) {
    case RecoveryPhase::BACKUP:
      r.goal.x = -1.0;
      if (elapsed > 2.0) {
        recovery_phase_ = RecoveryPhase::ROTATE;
        recovery_phase_start_ = ros::WallTime::now();
        ROS_INFO("Recovery: ROTATE 3s...");
      }
      break;
    case RecoveryPhase::ROTATE:
      r.goal.x = -2.0;
      if (elapsed > 3.0) {
        recovery_phase_ = RecoveryPhase::DONE;
        ROS_INFO("Recovery complete.");
        goal_manager_.markFailed(current_goal_);
        nav_monitor_.reset();
        transitionTo(State::DETECT_FRONTIER);
      }
      break;
    case RecoveryPhase::DONE: break;
  }
}

// ── External notifications ────────────────────────────────────
void ExplorationFSM::onPathReceived() {
  if (state_ == State::PLAN_PATH) {
    goal_start_time_ = ros::WallTime::now();
    transitionTo(State::FOLLOW_PATH);
  }
}

void ExplorationFSM::onPathFailed() {
  if (state_ == State::PLAN_PATH) {
    retry_count_++;
    if (retry_count_ >= config_.max_retries) {
      goal_manager_.markFailed(current_goal_);
      retry_count_ = 0;
      transitionTo(State::SELECT_GOAL);
    } else {
      transitionTo(State::REPLAN);
    }
  }
}

}  // namespace warehouse_exploration
