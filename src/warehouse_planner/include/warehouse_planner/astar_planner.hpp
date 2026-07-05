#ifndef WAREHOUSE_PLANNER_ASTAR_PLANNER_HPP_
#define WAREHOUSE_PLANNER_ASTAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

#include "warehouse_utils/grid_map.hpp"

namespace warehouse_planner {

/// A* global path planner on an occupancy grid.
///
/// Inputs:
///   - GridMap reference (already loaded with occupancy + inflation data)
///   - Start and goal in world coordinates (meters)
///
/// Output:
///   - Path as a vector of WorldPoint (empty = no path found)
///
/// Algorithm:
///   - 8-connected grid search
///   - Euclidean-distance heuristic (weighted)
///   - Uses inflated cost from GridMap for collision checking
///   - Priority queue (min-heap) for open set
class AStarPlanner {
public:
  /// Result type for a single planning call.
  struct Result {
    std::vector<warehouse_utils::WorldPoint> path;  ///< Empty if no path found
    int    nodes_expanded   = 0;   ///< Number of nodes expanded during search
    double path_length_m    = 0.0; ///< Total path length in meters
    double planning_time_ms = 0.0; ///< Wall-clock planning time (milliseconds)
    bool   success          = false;
  };

  /// Configuration for the planner.
  struct Config {
    double heuristic_weight   = 1.0;   ///< 1.0 = optimal, >1 = faster but suboptimal
    double collision_margin_m = 0.1;   ///< Extra margin around inflated obstacles
  };

  // ── Construction ────────────────────────────────────────
  explicit AStarPlanner(const Config& cfg);
  AStarPlanner() = default;

  // ── Planning ────────────────────────────────────────────
  /// Plan a path from start to goal using the given grid map.
  /// Returns a Result with path (empty on failure) and statistics.
  Result plan(const warehouse_utils::GridMap& grid,
              const warehouse_utils::WorldPoint& start,
              const warehouse_utils::WorldPoint& goal) const;

  /// Convenience: plan using grid coordinates.
  Result planGrid(const warehouse_utils::GridMap& grid,
                  const warehouse_utils::GridCell& start,
                  const warehouse_utils::GridCell& goal) const;

  // ── Accessors ───────────────────────────────────────────
  const Config& config() const { return config_; }

private:
  // ── Internal types ──────────────────────────────────────
  struct OpenNode {
    double f_score;     ///< f = g + h
    double g_score;     ///< cost from start
    int    row;
    int    col;

    // Min-heap: smaller f_score has higher priority.
    // Tie-break on g_score (larger g = closer to goal).
    bool operator<(const OpenNode& o) const {
      if (std::abs(f_score - o.f_score) > 1e-9) {
        return f_score > o.f_score;   // min-heap → greater goes to bottom
      }
      return g_score < o.g_score;    // larger g gets priority on tie
    }
  };

  /// Hash for GridCell to use in unordered_map.
  struct CellHash {
    std::size_t operator()(const warehouse_utils::GridCell& c) const {
      return static_cast<std::size_t>(c.row) * 65537 +
             static_cast<std::size_t>(c.col);
    }
  };

  /// Equality for GridCell.
  struct CellEqual {
    bool operator()(const warehouse_utils::GridCell& a,
                    const warehouse_utils::GridCell& b) const {
      return a.row == b.row && a.col == b.col;
    }
  };

  // ── Helpers ─────────────────────────────────────────────
  double heuristic(const warehouse_utils::GridCell& a,
                   const warehouse_utils::GridCell& b) const;

  bool isTraversable(const warehouse_utils::GridMap& grid,
                     int row, int col) const;

  void reconstructPath(
      const std::unordered_map<warehouse_utils::GridCell,
                               warehouse_utils::GridCell,
                               CellHash, CellEqual>& came_from,
      warehouse_utils::GridCell current,
      std::vector<warehouse_utils::GridCell>& out) const;

  // ── Data ────────────────────────────────────────────────
  Config config_;
};

}  // namespace warehouse_planner

#endif  // WAREHOUSE_PLANNER_ASTAR_PLANNER_HPP_
