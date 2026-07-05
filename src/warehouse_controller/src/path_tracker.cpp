#include "warehouse_controller/path_tracker.hpp"
#include <cmath>
#include <limits>

namespace warehouse_controller {

using warehouse_utils::WorldPoint;

PathTracker::PathTracker(const Config& cfg) : config_(cfg) {}

PathTracker::Result PathTracker::track(
    const std::vector<WorldPoint>& path,
    const WorldPoint& robot_pos) const {

  Result result;
  if (path.empty()) return result;
  if (path.size() == 1) {
    result.lookahead_point = path[0];
    result.closest_index   = 0;
    result.valid           = true;
    return result;
  }

  // Find closest point.
  result.closest_index = findClosestIndex(path, robot_pos);

  // Compute cross-track error (distance from robot to closest path point).
  const auto& closest = path[result.closest_index];
  const double dx = robot_pos.x - closest.x;
  const double dy = robot_pos.y - closest.y;
  result.cross_track_error = std::sqrt(dx * dx + dy * dy);

  // Look ahead: walk forward along the path until accumulated distance
  // exceeds lookahead_distance_m, or we reach the end.
  double accumulated = 0.0;
  int idx = result.closest_index;

  for (int i = result.closest_index; i < static_cast<int>(path.size()) - 1; ++i) {
    const double seg_dx = path[i + 1].x - path[i].x;
    const double seg_dy = path[i + 1].y - path[i].y;
    const double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

    if (accumulated + seg_len >= config_.lookahead_distance_m) {
      // Interpolate on this segment.
      const double remaining = config_.lookahead_distance_m - accumulated;
      const double t = (seg_len > 1e-9) ? remaining / seg_len : 0.0;
      result.lookahead_point.x = path[i].x + t * seg_dx;
      result.lookahead_point.y = path[i].y + t * seg_dy;
      result.valid = true;
      return result;
    }

    accumulated += seg_len;
    idx = i + 1;
  }

  // Reached end of path — return last point.
  result.lookahead_point = path.back();
  result.valid = true;
  return result;
}

int PathTracker::findClosestIndex(
    const std::vector<WorldPoint>& path,
    const WorldPoint& pos) const {

  int    best_idx  = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    const double dx = path[i].x - pos.x;
    const double dy = path[i].y - pos.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best_idx  = i;
    }
  }
  return best_idx;
}

}  // namespace warehouse_controller
