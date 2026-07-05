#include "warehouse_exploration/frontier_detector.hpp"

namespace warehouse_exploration {

using warehouse_utils::GridMap;
using warehouse_utils::GridCell;

std::vector<GridCell> FrontierDetector::findFrontierCells(
    const GridMap& grid) const {

  std::vector<GridCell> frontiers;
  const int h = grid.height();
  const int w = grid.width();

  static constexpr int DR[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int DC[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (!grid.isFree(r, c)) continue;
      if (!grid.isPassable(r, c)) continue;

      for (int k = 0; k < 8; ++k) {
        const int nr = r + DR[k];
        const int nc = c + DC[k];
        if (grid.isValid(nr, nc) && grid.isUnknown(nr, nc)) {
          frontiers.emplace_back(GridCell{r, c});
          break;
        }
      }
    }
  }

  return frontiers;
}

}  // namespace warehouse_exploration
