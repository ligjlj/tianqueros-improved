#include "warehouse_utils/grid_map.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <utility>
#include <stdexcept>
#include <ros/console.h>

namespace warehouse_utils {

// ═══════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════

GridMap::GridMap(const Config& cfg)
  : config_(cfg)
{}

// ═══════════════════════════════════════════════════════════════
// Loading
// ═══════════════════════════════════════════════════════════════

void GridMap::loadFromMsg(const nav_msgs::OccupancyGrid& msg) {
  loadFromRaw(
    msg.data,
    msg.info.width,
    msg.info.height,
    msg.info.resolution,
    msg.info.origin.position.x,
    msg.info.origin.position.y
  );
}

void GridMap::loadFromRaw(const std::vector<int8_t>& data,
                          int width, int height,
                          double resolution,
                          double origin_x, double origin_y) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("GridMap: width and height must be positive.");
  }
  if (static_cast<int>(data.size()) != width * height) {
    throw std::invalid_argument(
      "GridMap: data size (" + std::to_string(data.size()) +
      ") != width*height (" + std::to_string(width * height) + ").");
  }
  if (resolution <= 0.0) {
    throw std::invalid_argument("GridMap: resolution must be positive.");
  }

  width_      = width;
  height_     = height;
  resolution_ = resolution;
  origin_x_   = origin_x;
  origin_y_   = origin_y;
  raw_grid_   = data;

  ROS_DEBUG_STREAM("GridMap loaded: " << width_ << "x" << height_
                   << " @ " << resolution_ << "m/cell");

  // Automatically inflate after loading.
  inflate();
}

// ═══════════════════════════════════════════════════════════════
// Coordinate Conversion
// ═══════════════════════════════════════════════════════════════

GridCell GridMap::worldToGrid(double wx, double wy) const {
  return GridCell{
    static_cast<int>((wy - origin_y_) / resolution_),
    static_cast<int>((wx - origin_x_) / resolution_)
  };
}

WorldPoint GridMap::gridToWorld(int row, int col) const {
  // Return the center of the cell.
  return WorldPoint{
    (col + 0.5) * resolution_ + origin_x_,
    (row + 0.5) * resolution_ + origin_y_
  };
}

// ═══════════════════════════════════════════════════════════════
// Cell State Queries
// ═══════════════════════════════════════════════════════════════

bool GridMap::isValid(int row, int col) const {
  return row >= 0 && row < height_ && col >= 0 && col < width_;
}

CellState GridMap::getState(int row, int col) const {
  if (!isValid(row, col)) {
    return CellState::UNKNOWN;
  }
  int8_t val = raw_grid_[index(row, col)];
  if (val == OCCUPANCY_UNKNOWN) {
    return CellState::UNKNOWN;
  }
  if (val > OCCUPANCY_OCCUPIED_THRESHOLD) {
    return CellState::OCCUPIED;
  }
  return CellState::FREE;
}

bool GridMap::isFree(int row, int col) const {
  return getState(row, col) == CellState::FREE;
}

bool GridMap::isOccupied(int row, int col) const {
  return getState(row, col) == CellState::OCCUPIED;
}

bool GridMap::isUnknown(int row, int col) const {
  return getState(row, col) == CellState::UNKNOWN;
}

bool GridMap::isFreeFootprint(int row, int col) const {
  const int rr = config_.robot_radius_cells();
  for (int dr = -rr; dr <= rr; ++dr) {
    for (int dc = -rr; dc <= rr; ++dc) {
      // Only check cells within the circular footprint
      if (dr * dr + dc * dc > rr * rr) {
        continue;
      }
      int nr = row + dr;
      int nc = col + dc;
      if (!isValid(nr, nc) || isOccupied(nr, nc)) {
        return false;
      }
    }
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════
// Inflation
// ═══════════════════════════════════════════════════════════════

void GridMap::inflate() {
  const int total_cells = width_ * height_;
  const int inflate_r = config_.inflation_radius_cells();

  // Initialise inflated grid: infinite (free) everywhere.
  inflated_grid_.assign(total_cells, std::numeric_limits<double>::infinity());

  if (inflate_r <= 0) {
    // No inflation requested — everything is free.
    for (int i = 0; i < total_cells; ++i) {
      if (getState(i / width_, i % width_) == CellState::OCCUPIED) {
        inflated_grid_[i] = 254.0;  // Lethal
      } else {
        inflated_grid_[i] = 0.0;
      }
    }
    return;
  }

  // BFS queue: (row, col).
  std::queue<std::pair<int, int>> frontier;

  // Seed: push all occupied cells with distance 0.
  for (int r = 0; r < height_; ++r) {
    for (int c = 0; c < width_; ++c) {
      if (getState(r, c) == CellState::OCCUPIED) {
        inflated_grid_[index(r, c)] = 0.0;
        frontier.emplace(r, c);
      }
    }
  }

  // 8-connected BFS expansion.
  const int dr8[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  const int dc8[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  while (!frontier.empty()) {
    auto [r, c] = frontier.front();
    frontier.pop();

    const double current_dist = inflated_grid_[index(r, c)];
    const double next_dist = current_dist + 1.0;  // cell-distance

    // Stop expanding beyond inflation radius.
    if (current_dist >= static_cast<double>(inflate_r)) {
      continue;
    }

    for (int k = 0; k < 8; ++k) {
      const int nr = r + dr8[k];
      const int nc = c + dc8[k];

      if (!isValid(nr, nc)) {
        continue;
      }

      const int ni = index(nr, nc);

      // Only update if we found a shorter path.
      if (next_dist < inflated_grid_[ni]) {
        // Diagonal steps cost sqrt(2) in cell distance.
        const double step_cost = (k == 1 || k == 3 || k == 4 || k == 6) ? 1.0 : std::sqrt(2.0);
        const double actual_dist = current_dist + step_cost;

        if (actual_dist < inflated_grid_[ni]) {
          inflated_grid_[ni] = actual_dist;
          frontier.emplace(nr, nc);
        }
      }
    }
  }

  // Convert distance → cost.
  // cost = 254 * (1 - dist / inflate_r)^2  clamped to [0, 254]
  // Cells with infinite distance (never reached) = 0 (free).
  // Occupied cells stay at 254 (lethal).
  for (int i = 0; i < total_cells; ++i) {
    const double d = inflated_grid_[i];
    if (std::isinf(d)) {
      inflated_grid_[i] = 0.0;
    } else if (d >= static_cast<double>(inflate_r)) {
      inflated_grid_[i] = 0.0;
    } else {
      // Quadratic decay: close to obstacle = high cost.
      const double ratio = 1.0 - d / static_cast<double>(inflate_r);
      inflated_grid_[i] = 254.0 * ratio * ratio;
    }
    // Clamp.
    if (inflated_grid_[i] > 254.0) inflated_grid_[i] = 254.0;
  }

  ROS_DEBUG_STREAM("GridMap inflated: radius=" << inflate_r
                   << " cells (" << config_.inflation_radius_m << " m)");
}

double GridMap::getInflatedCost(int row, int col) const {
  if (!isValid(row, col)) {
    return 254.0;  // Out of bounds = lethal.
  }
  return inflated_grid_[index(row, col)];
}

bool GridMap::isPassable(int row, int col, double lethal_threshold) const {
  return getInflatedCost(row, col) < lethal_threshold;
}

}  // namespace warehouse_utils
