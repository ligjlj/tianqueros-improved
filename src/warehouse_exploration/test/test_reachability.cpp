#include <gtest/gtest.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_exploration/reachability_checker.hpp"

using namespace warehouse_utils;
using namespace warehouse_exploration;

class ReachTest : public ::testing::Test {
protected:
  void SetUp() override {
    GridMap::Config gcfg;
    gcfg.inflation_radius_m = 0.1;  // Small inflation for test
    gcfg.robot_radius_m     = 0.05;
    gcfg.resolution_m       = 0.05;
    grid_ = GridMap(gcfg);
  }
  GridMap grid_;
};

TEST_F(ReachTest, SameCellIsReachable) {
  std::vector<int8_t> data(25, 0);
  grid_.loadFromRaw(data, 5, 5, 0.05, 0.0, 0.0);
  ReachabilityChecker rc;
  EXPECT_TRUE(rc.isReachable(grid_, {2,2}, {2,2}));
}

TEST_F(ReachTest, StraightPathReachable) {
  std::vector<int8_t> data(25, 0);
  grid_.loadFromRaw(data, 5, 5, 0.05, 0.0, 0.0);
  ReachabilityChecker rc;
  EXPECT_TRUE(rc.isReachable(grid_, {0,0}, {4,4}));
}

TEST_F(ReachTest, BlockedByWall) {
  // 5x5: solid wall at row 2.
  std::vector<int8_t> data(25, 0);
  for (int c = 0; c < 5; ++c) data[2*5 + c] = 100;
  grid_.loadFromRaw(data, 5, 5, 0.05, 0.0, 0.0);

  ReachabilityChecker rc;
  EXPECT_FALSE(rc.isReachable(grid_, {1,2}, {3,2}));
}

TEST_F(ReachTest, GapInWallIsReachable) {
  // 5x5 grid: wall at row 2 with gap at col 2.
  std::vector<int8_t> data(25, 0);
  data[2*5 + 0] = 100;
  data[2*5 + 1] = 100;
  data[2*5 + 3] = 100;
  data[2*5 + 4] = 100;
  grid_.loadFromRaw(data, 5, 5, 0.05, 0.0, 0.0);

  ReachabilityChecker rc;
  EXPECT_TRUE(rc.isReachable(grid_, {1,2}, {3,2}));
}

TEST_F(ReachTest, FilterReachableClusters) {
  // 5x5: solid wall at row 2.
  std::vector<int8_t> data(25, 0);
  for (int c = 0; c < 5; ++c) data[2*5 + c] = 100;
  grid_.loadFromRaw(data, 5, 5, 0.05, 0.0, 0.0);

  std::vector<GridCell> robot_pos = {{1, 2}};
  std::vector<std::vector<GridCell>> clusters = {
    {{1,1}},   // Cluster 0: above wall → reachable
    {{3,3}},   // Cluster 1: below wall → NOT reachable
  };

  ReachabilityChecker rc;
  auto reachable = rc.filterReachable(grid_, robot_pos, clusters);

  ASSERT_EQ(reachable.size(), 1u);
  EXPECT_EQ(reachable[0], 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
