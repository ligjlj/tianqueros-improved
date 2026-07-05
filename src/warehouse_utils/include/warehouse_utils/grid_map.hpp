#ifndef WAREHOUSE_UTILS_GRID_MAP_HPP_
#define WAREHOUSE_UTILS_GRID_MAP_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

#include <nav_msgs/OccupancyGrid.h>

namespace warehouse_utils {

/// Cell state classification based on OccupancyGrid conventions.
/// Unknown:  -1  (nav_msgs::OccupancyGrid convention)
/// Free:      0..threshold
/// Occupied:  threshold+1..100
enum class CellState {
  FREE      = 0,
  OCCUPIED  = 1,
  UNKNOWN   = 2,
};

/// Grid cell in pixel coordinates (row, column).
struct GridCell {
  int row;
  int col;

  bool operator==(const GridCell& o) const {
    return row == o.row && col == o.col;
  }
};

/// World point in meters.
struct WorldPoint {
  double x;
  double y;
};

/// 2D occupancy grid with inflation support.
///
/// Responsibilities:
///   - Load OccupancyGrid ROS messages into internal matrix
///   - Convert between world (meters) and grid (row, col) coordinates
///   - Query cell state: free / occupied / unknown
///   - Compute inflation layer from obstacles
///   - Query inflated cost for path planning
///
/// This class is ROS-agnostic — it does NOT subscribe to topics.
/// The caller is responsible for passing the OccupancyGrid message.
class GridMap {
public:
  // ── Constants ────────────────────────────────────────────
  static constexpr int8_t  OCCUPANCY_OCCUPIED_THRESHOLD = 50;   ///< >50 → occupied
  static constexpr int8_t  OCCUPANCY_UNKNOWN            = -1;
  static constexpr int8_t  OCCUPANCY_FREE_MIN            = 0;
  static constexpr int8_t  OCCUPANCY_FREE_MAX            = 50;

  // ── Configuration ───────────────────────────────────────
  struct Config {
    double inflation_radius_m  = 1.0;   ///< Inflation radius in meters
    double robot_radius_m      = 0.5;   ///< Robot footprint radius in meters
    double resolution_m        = 0.05;  ///< Map resolution (m/cell)
    int    inflation_radius_cells() const {
      return static_cast<int>(std::ceil(inflation_radius_m / resolution_m));
    }
    int    robot_radius_cells() const {
      return static_cast<int>(std::ceil(robot_radius_m / resolution_m));
    }
  };

  // ── Construction ────────────────────────────────────────
  GridMap() = default;
  explicit GridMap(const Config& cfg);

  /// Load from a nav_msgs::OccupancyGrid.
  /// Resets the map and rebuilds the inflation layer.
  void loadFromMsg(const nav_msgs::OccupancyGrid& msg);

  /// Load from raw data for testing (no ROS dependency).
  void loadFromRaw(const std::vector<int8_t>& data,
                   int width, int height,
                   double resolution,
                   double origin_x, double origin_y);

  // ── Coordinate Conversion ───────────────────────────────
  GridCell  worldToGrid(double wx, double wy) const;
  WorldPoint gridToWorld(int row, int col) const;

  // ── Cell State Queries ──────────────────────────────────
  bool isValid(int row, int col) const;
  bool isFree(int row, int col) const;
  bool isOccupied(int row, int col) const;
  bool isUnknown(int row, int col) const;
  CellState getState(int row, int col) const;

  /// Check if a grid cell is free considering the robot footprint.
  /// Scans all cells within robot_radius_cells_ of (row, col).
  bool isFreeFootprint(int row, int col) const;

  // ── Inflation ───────────────────────────────────────────
  /// Build the inflation layer. Creates a distance-based cost gradient
  /// radiating from every occupied cell up to inflation_radius_cells_.
  /// Must be called after loadFromMsg / loadFromRaw, or re-run if
  /// the occupancy grid changes.
  void inflate();

  /// Get the inflated cost at a grid cell.
  /// - 0 means completely free
  /// - Values ramp up near obstacles
  /// - Returns a high value for cells inside an obstacle
  double getInflatedCost(int row, int col) const;

  /// Check whether a cell is passable according to the inflated map.
  /// Passable = cost < lethality threshold.
  bool isPassable(int row, int col, double lethal_threshold = 100.0) const;

  // ── Accessors ───────────────────────────────────────────
  int    width()      const { return width_; }
  int    height()     const { return height_; }
  double resolution() const { return resolution_; }
  double originX()    const { return origin_x_; }
  double originY()    const { return origin_y_; }
  const std::vector<int8_t>&  rawGrid()   const { return raw_grid_; }
  const std::vector<double>&  inflatedGrid() const { return inflated_grid_; }
  const Config& config() const { return config_; }

private:
  // ── Internal Helpers ────────────────────────────────────
  int  index(int row, int col) const { return row * width_ + col; }

  // ── Data ────────────────────────────────────────────────
  Config config_;

  int    width_       = 0;
  int    height_      = 0;
  double resolution_  = 0.05;
  double origin_x_    = 0.0;
  double origin_y_    = 0.0;

  /// Raw occupancy grid (flat row-major).
  /// Values as per OccupancyGrid convention: -1=unknown, 0=free, 100=occupied.
  std::vector<int8_t> raw_grid_;

  /// Inflated cost grid (flat row-major).
  /// 0 = free, ramping up near obstacles.
  std::vector<double> inflated_grid_;
};

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS_GRID_MAP_HPP_
