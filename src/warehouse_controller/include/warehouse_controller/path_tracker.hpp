#ifndef WAREHOUSE_CONTROLLER_PATH_TRACKER_HPP_
#define WAREHOUSE_CONTROLLER_PATH_TRACKER_HPP_

#include <vector>
#include "warehouse_utils/grid_map.hpp"

namespace warehouse_controller {

/// Pure Pursuit path tracker.
///
/// Given a global path (ordered waypoints) and the robot's current pose,
/// finds the closest point on the path, then looks ahead by a configurable
/// lookahead distance. The resulting point becomes the sub-goal for DWA.
class PathTracker {
public:
  struct Config {
    double lookahead_distance_m = 0.5;   ///< How far ahead to look (m)
    double path_error_threshold_m = 0.1; ///< Max cross-track error (m)
  };

  struct Result {
    warehouse_utils::WorldPoint lookahead_point;
    int    closest_index    = -1;   ///< Index of closest path point
    double cross_track_error = 0.0; ///< Distance from robot to path (m)
    bool   valid            = false;
  };

  explicit PathTracker(const Config& cfg);
  PathTracker() = default;

  /// Find the lookahead point on the path from the robot's current position.
  Result track(const std::vector<warehouse_utils::WorldPoint>& path,
               const warehouse_utils::WorldPoint& robot_pos) const;

  const Config& config() const { return config_; }

private:
  /// Find the index of the closest point on the path to (x, y).
  int findClosestIndex(const std::vector<warehouse_utils::WorldPoint>& path,
                       const warehouse_utils::WorldPoint& pos) const;

  Config config_;
};

}  // namespace warehouse_controller

#endif
