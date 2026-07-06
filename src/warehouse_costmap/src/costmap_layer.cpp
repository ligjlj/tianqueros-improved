#include "warehouse_costmap/costmap_layer.hpp"
#include <queue>
#include <cmath>
#include <limits>

namespace warehouse_costmap {

// ═══════════════════════════════════════════════════════════════
// RawCostmap
// ═══════════════════════════════════════════════════════════════

void RawCostmap::loadFromMsg(const nav_msgs::OccupancyGrid& msg) {
  w_ = msg.info.width;
  h_ = msg.info.height;
  res_ = msg.info.resolution;
  ox_ = msg.info.origin.position.x;
  oy_ = msg.info.origin.position.y;
  data_ = msg.data;
}

double RawCostmap::getCost(int row, int col) const {
  if (!isValid(row, col)) return 254;
  int8_t v = data_[row * w_ + col];
  if (v < 0) return -1;       // unknown
  if (v > 50) return 254;     // occupied
  return 0;                    // free
}

int8_t RawCostmap::getRaw(int row, int col) const {
  if (!isValid(row, col)) return -1;
  return data_[row * w_ + col];
}

// ═══════════════════════════════════════════════════════════════
// InflatedCostmap
// ═══════════════════════════════════════════════════════════════

void InflatedCostmap::buildFrom(const RawCostmap& raw, double inflation_radius_m) {
  w_ = raw.width(); h_ = raw.height();
  res_ = raw.resolution(); ox_ = raw.originX(); oy_ = raw.originY();
  int total = w_ * h_;
  cost_.assign(total, std::numeric_limits<double>::infinity());

  int inflate_cells = static_cast<int>(std::ceil(inflation_radius_m / res_));
  if (inflate_cells <= 0) { inflate_cells = 1; }

  // Seed: push all occupied cells.
  std::queue<std::pair<int,int>> q;
  for (int r = 0; r < h_; ++r)
    for (int c = 0; c < w_; ++c)
      if (raw.getCost(r, c) > 253) { cost_[r*w_+c] = 0; q.emplace(r,c); }

  static const int DR[8] = {-1,-1,-1,0,0,1,1,1};
  static const int DC[8] = {-1,0,1,-1,1,-1,0,1};

  while (!q.empty()) {
    auto [r,c] = q.front(); q.pop();
    double cur = cost_[r*w_+c];
    if (cur >= inflate_cells) continue;
    for (int k = 0; k < 8; ++k) {
      int nr = r+DR[k], nc = c+DC[k];
      if (nr<0||nr>=h_||nc<0||nc>=w_) continue;
      double step = (k==1||k==3||k==4||k==6) ? 1.0 : std::sqrt(2.0);
      double nd = cur + step;
      if (nd < cost_[nr*w_+nc]) { cost_[nr*w_+nc] = nd; q.emplace(nr,nc); }
    }
  }

  // Convert distance → cost.
  for (int i = 0; i < total; ++i) {
    const double d = cost_[i];
    if (std::isinf(d)) cost_[i] = 0;
    else if (d >= inflate_cells) cost_[i] = 0;
    else { double r = 1.0 - d/inflate_cells; cost_[i] = 254.0 * r * r; }
    if (cost_[i] > 254) cost_[i] = 254;
  }
}

double InflatedCostmap::getCost(int row, int col) const {
  if (row<0||row>=h_||col<0||col>=w_) return 254;
  return cost_[row * w_ + col];
}

}  // namespace warehouse_costmap
