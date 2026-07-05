/// Unit tests for FrontierDetector + FrontierClusterer.

#include <gtest/gtest.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/frontier_detector.hpp"
#include "warehouse_exploration/frontier_cluster.hpp"

using namespace warehouse_utils;
using namespace warehouse_exploration;

class FrontierTest : public ::testing::Test {
protected:
  void SetUp() override {
    GridMap::Config gcfg;
    gcfg.inflation_radius_m = 1.0;
    gcfg.robot_radius_m     = 0.5;
    gcfg.resolution_m       = 0.05;
    grid_ = GridMap(gcfg);
  }

  void loadHalfKnown() {
    std::vector<int8_t> data(400, -1);
    for (int r = 0; r < 20; ++r)
      for (int c = 0; c < 10; ++c)
        data[r * 20 + c] = 0;
    grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  }

  GridMap grid_;
};

TEST_F(FrontierTest, NoFrontiersOnAllFree) {
  std::vector<int8_t> data(400, 0);
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  FrontierDetector det;
  auto cells = det.findFrontierCells(grid_);
  EXPECT_TRUE(cells.empty());
}

TEST_F(FrontierTest, NoFrontiersOnAllUnknown) {
  std::vector<int8_t> data(400, -1);
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  FrontierDetector det;
  auto cells = det.findFrontierCells(grid_);
  EXPECT_TRUE(cells.empty());
}

TEST_F(FrontierTest, FrontiersOnBoundary) {
  loadHalfKnown();
  FrontierDetector det;
  auto cells = det.findFrontierCells(grid_);
  EXPECT_GE(cells.size(), 20u);  // 20 rows × 1 col boundary

  auto clusters = FrontierClusterer().cluster(cells);
  EXPECT_FALSE(clusters.empty());
}

TEST_F(FrontierTest, ClustersHaveCentroids) {
  loadHalfKnown();
  FrontierDetector det;
  auto cells = det.findFrontierCells(grid_);
  auto clusters = FrontierClusterer().cluster(cells);
  ASSERT_FALSE(clusters.empty());

  for (const auto& cl : clusters) {
    EXPECT_GT(cl.size, 0);
    EXPECT_GE(cl.center.row, cl.min_row);
    EXPECT_LE(cl.center.row, cl.max_row);
  }
}

TEST_F(FrontierTest, BoundingBoxCorrect) {
  // Create a small known patch surrounded by unknown.
  std::vector<int8_t> data(400, -1);
  for (int r = 8; r <= 12; ++r)
    for (int c = 8; c <= 12; ++c)
      data[r * 20 + c] = 0;

  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  FrontierDetector det;
  auto cells = det.findFrontierCells(grid_);
  auto clusters = FrontierClusterer().cluster(cells);
  ASSERT_EQ(clusters.size(), 1u);

  // Frontier cells are on the border of the known patch.
  // Bounding box should encompass rows 8-12 and cols 8-12.
  EXPECT_LE(clusters[0].min_row, 8);
  EXPECT_GE(clusters[0].max_row, 12);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
