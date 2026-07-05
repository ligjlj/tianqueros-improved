#include "warehouse_exploration/frontier_cluster.hpp"
#include <algorithm>

namespace warehouse_exploration {

std::vector<FrontierCluster> FrontierClusterer::cluster(
    const std::vector<GridCell>& cells) const {

  std::unordered_set<GridCell, CellHash> unvisited(cells.begin(), cells.end());
  std::vector<FrontierCluster> clusters;

  static constexpr int DR[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  static constexpr int DC[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

  while (!unvisited.empty()) {
    FrontierCluster cl;
    std::queue<GridCell> q;

    const GridCell seed = *unvisited.begin();
    q.push(seed);
    unvisited.erase(seed);

    cl.min_row = cl.max_row = seed.row;
    cl.min_col = cl.max_col = seed.col;

    while (!q.empty()) {
      const GridCell cur = q.front();
      q.pop();

      cl.cells.push_back(cur);
      ++cl.size;

      // Update bounding box.
      cl.min_row = std::min(cl.min_row, cur.row);
      cl.max_row = std::max(cl.max_row, cur.row);
      cl.min_col = std::min(cl.min_col, cur.col);
      cl.max_col = std::max(cl.max_col, cur.col);

      for (int k = 0; k < 8; ++k) {
        const GridCell nb{cur.row + DR[k], cur.col + DC[k]};
        auto it = unvisited.find(nb);
        if (it != unvisited.end()) {
          q.push(nb);
          unvisited.erase(it);
        }
      }
    }

    // Compute centroid.
    double sum_r = 0, sum_c = 0;
    for (const auto& c : cl.cells) {
      sum_r += c.row;
      sum_c += c.col;
    }
    cl.center = GridCell{
      static_cast<int>(sum_r / cl.size),
      static_cast<int>(sum_c / cl.size)
    };

    clusters.push_back(std::move(cl));
  }

  return clusters;
}

}  // namespace warehouse_exploration
