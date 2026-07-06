/// Unit tests for GoalSelector weighted scoring.

#include <gtest/gtest.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/frontier_detector.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"
#include "warehouse_exploration/goal_selector.hpp"

using namespace warehouse_utils;
using namespace warehouse_exploration;

class GoalTest : public ::testing::Test {
protected:
  void SetUp() override {
    GoalSelector::Config cfg;
    cfg.weight_distance     = 1.0;
    cfg.weight_information  = 2.0;
    cfg.weight_size         = 1.5;
    cfg.min_cluster_size    = 5;
    cfg.min_goal_distance_m = 0.0;   // Disable min distance for small test grids
    selector_ = GoalSelector(cfg);

    GridMap::Config gcfg;
    gcfg.inflation_radius_m = 1.0;
    gcfg.robot_radius_m     = 0.5;
    gcfg.resolution_m       = 0.05;
    grid_ = GridMap(gcfg);
  }

  GoalSelector selector_;
  GridMap      grid_;
};

TEST_F(GoalTest, EmptyClustersReturnsInvalid) {
  auto result = selector_.select({}, WorldPoint{0, 0}, grid_);
  EXPECT_EQ(result.cluster_index, -1);
}

TEST_F(GoalTest, PrefersLargerCluster) {
  std::vector<int8_t> data(400, -1);
  for (int r = 0; r < 20; ++r)
    for (int c = 0; c < 10; ++c)
      data[r * 20 + c] = 0;
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  FrontierDetector::Config fcfg;
  fcfg.min_frontier_size = 1;
  FrontierDetector det(fcfg);
  auto cells = det.findFrontierCells(grid_);
  auto clusters = FrontierClusterer().cluster(cells);
  ASSERT_FALSE(clusters.empty());

  auto result = selector_.select(clusters, WorldPoint{0, 0}, grid_);
  EXPECT_GE(result.cluster_index, 0);
  EXPECT_GT(result.score, 0.0);
}

TEST_F(GoalTest, PrefersCloserCluster) {
  std::vector<int8_t> data(400, -1);
  // Two known regions: near (0,0) and far (0,15).
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < 5; ++c)
      data[r * 20 + c] = 0;  // near region
  for (int r = 10; r < 20; ++r)
    for (int c = 15; c < 20; ++c)
      data[r * 20 + c] = 0;  // far region

  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  FrontierDetector det(FrontierDetector::Config{1});
  auto cells = det.findFrontierCells(grid_);
  auto clusters = FrontierClusterer().cluster(cells);

  // Robot near the first region.
  auto result = selector_.select(clusters, WorldPoint{0.1, 0.1}, grid_);
  EXPECT_GE(result.cluster_index, 0);
  // The selected goal should be closer to (0.1, 0.1) than to the far region.
  EXPECT_LT(result.goal.x, 1.0);  // near region is around x=0.25
}

TEST_F(GoalTest, FiltersSmallClusters) {
  std::vector<int8_t> data(400, -1);
  // Large known region.
  for (int r = 0; r < 20; ++r)
    for (int c = 0; c < 15; ++c)
      data[r * 20 + c] = 0;
  // Tiny isolated free cell (size 1 cluster).
  data[18 * 20 + 18] = 0;

  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  FrontierDetector det(FrontierDetector::Config{1});
  auto cells = det.findFrontierCells(grid_);
  auto clusters = FrontierClusterer().cluster(cells);

  auto result = selector_.select(clusters, WorldPoint{0, 0}, grid_);
  EXPECT_GE(result.cluster_index, 0);

  // The goal should NOT be at the tiny isolated cell.
  auto cell = grid_.worldToGrid(result.goal.x, result.goal.y);
  EXPECT_NE(cell.row * 20 + cell.col, 18 * 20 + 18);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
