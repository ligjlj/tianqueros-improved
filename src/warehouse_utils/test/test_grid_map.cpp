#include <gtest/gtest.h>
#include "warehouse_utils/grid_map.hpp"

using namespace warehouse_utils;

// ═══════════════════════════════════════════════════════════════
// Fixture: 10x10 simple grid for most tests
// ═══════════════════════════════════════════════════════════════

class GridMapTest : public ::testing::Test {
protected:
  void SetUp() override {
    GridMap::Config cfg;
    cfg.resolution_m       = 0.05;
    cfg.robot_radius_m     = 0.05;   // 1 cell (smaller for map boundary testing)
    cfg.inflation_radius_m = 0.25;   // 5 cells
    map_ = GridMap(cfg);
  }

  /// Load a 10x10 grid with a single obstacle at (5,5).
  void loadSimpleGrid() {
    // 10x10: row 0-9, col 0-9. obstacle at center (5,5).
    std::vector<int8_t> data(100, 0);  // all free
    data[5 * 10 + 5] = 100;            // occupied
    data[9 * 10 + 9] = -1;             // unknown at bottom-right
    map_.loadFromRaw(data, 10, 10, 0.05, -0.25, -0.25);
  }

  GridMap map_;
};

// ═══════════════════════════════════════════════════════════════
// Construction & Loading
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, DefaultConstruction) {
  GridMap m;
  EXPECT_EQ(m.width(), 0);
  EXPECT_EQ(m.height(), 0);
}

TEST_F(GridMapTest, LoadFromRawSetsDimensions) {
  std::vector<int8_t> data(100, 0);
  map_.loadFromRaw(data, 10, 10, 0.05, 0.0, 0.0);
  EXPECT_EQ(map_.width(), 10);
  EXPECT_EQ(map_.height(), 10);
  EXPECT_DOUBLE_EQ(map_.resolution(), 0.05);
  EXPECT_DOUBLE_EQ(map_.originX(), 0.0);
  EXPECT_DOUBLE_EQ(map_.originY(), 0.0);
}

TEST_F(GridMapTest, LoadFromRawThrowsOnBadSize) {
  std::vector<int8_t> data(50, 0);
  EXPECT_THROW(map_.loadFromRaw(data, 10, 10, 0.05, 0, 0), std::invalid_argument);
}

TEST_F(GridMapTest, LoadFromRawThrowsOnZeroDimension) {
  std::vector<int8_t> data;
  EXPECT_THROW(map_.loadFromRaw(data, 0, 10, 0.05, 0, 0), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════
// Coordinate Conversion
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, WorldToGridCenter) {
  loadSimpleGrid();
  // origin at (-0.25, -0.25), res=0.05
  // cell (5, 5) center world = (5+0.5)*0.05 - 0.25 = 0.025
  auto cell = map_.worldToGrid(0.025, 0.025);
  EXPECT_EQ(cell.row, 5);
  EXPECT_EQ(cell.col, 5);
}

TEST_F(GridMapTest, GridToWorldCenter) {
  loadSimpleGrid();
  auto wp = map_.gridToWorld(5, 5);
  EXPECT_DOUBLE_EQ(wp.x, (5 + 0.5) * 0.05 - 0.25);
  EXPECT_DOUBLE_EQ(wp.y, (5 + 0.5) * 0.05 - 0.25);
}

TEST_F(GridMapTest, WorldToGridRoundTrip) {
  loadSimpleGrid();
  auto wp = map_.gridToWorld(3, 7);
  auto cell = map_.worldToGrid(wp.x, wp.y);
  EXPECT_EQ(cell.row, 3);
  EXPECT_EQ(cell.col, 7);
}

// ═══════════════════════════════════════════════════════════════
// Cell State Queries
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, IsValidBoundaries) {
  loadSimpleGrid();
  EXPECT_TRUE(map_.isValid(0, 0));
  EXPECT_TRUE(map_.isValid(9, 9));
  EXPECT_FALSE(map_.isValid(-1, 0));
  EXPECT_FALSE(map_.isValid(0, -1));
  EXPECT_FALSE(map_.isValid(10, 0));
  EXPECT_FALSE(map_.isValid(0, 10));
}

TEST_F(GridMapTest, CellStates) {
  loadSimpleGrid();
  EXPECT_EQ(map_.getState(0, 0), CellState::FREE);
  EXPECT_EQ(map_.getState(5, 5), CellState::OCCUPIED);
  EXPECT_EQ(map_.getState(9, 9), CellState::UNKNOWN);
  // Out of bounds → UNKNOWN
  EXPECT_EQ(map_.getState(-1, -1), CellState::UNKNOWN);
}

TEST_F(GridMapTest, OccupancyThreshold) {
  std::vector<int8_t> data(25, 0);
  data[0] = 49;   // should be FREE (≤50)
  data[1] = 50;   // should be FREE (≤50)
  data[2] = 51;   // should be OCCUPIED (>50)
  data[3] = 100;  // should be OCCUPIED
  data[4] = -1;   // should be UNKNOWN
  map_.loadFromRaw(data, 5, 5, 0.05, 0, 0);

  EXPECT_TRUE(map_.isFree(0, 0));      // 49
  EXPECT_TRUE(map_.isFree(0, 1));      // 50
  EXPECT_TRUE(map_.isOccupied(0, 2));  // 51
  EXPECT_TRUE(map_.isOccupied(0, 3));  // 100
  EXPECT_TRUE(map_.isUnknown(0, 4));   // -1
}

// ═══════════════════════════════════════════════════════════════
// Footprint Check
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, FreeFootprintAwayFromObstacle) {
  loadSimpleGrid();
  // (1,1) is far from obstacle (5,5) AND at least 1 cell from all map edges.
  // Footprint radius=1, so needs 1-cell margin from boundary.
  EXPECT_TRUE(map_.isFreeFootprint(1, 1));
}

TEST_F(GridMapTest, OccupiedFootprintNearObstacle) {
  loadSimpleGrid();
  // Robot radius = 1 cell. (4,4) is 1 cell diagonally from (5,5).
  // Footprint at (4,4) covers (4,4) itself — safe.
  // Actually need to be within the radius to hit: (4,5) is adjacent to obstacle.
  EXPECT_FALSE(map_.isFreeFootprint(4, 5));
}

// ═══════════════════════════════════════════════════════════════
// Inflation Layer
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, InflateCreatesCostGradient) {
  loadSimpleGrid();
  // Obstacle at (5,5). Inflate radius = 5 cells.
  // Cell (5,5) itself = lethal (254).
  EXPECT_GT(map_.getInflatedCost(5, 5), 250.0);

  // Cell (0,0) = 8 cells away (>5) → completely free.
  EXPECT_DOUBLE_EQ(map_.getInflatedCost(0, 0), 0.0);

  // Cell (4,4) = 1 cell away → high cost but not lethal.
  double cost_4_4 = map_.getInflatedCost(4, 4);
  EXPECT_GT(cost_4_4, 0.0);
  EXPECT_LT(cost_4_4, 254.0);

  // Closer cells should have higher cost than farther cells.
  double cost_3_3 = map_.getInflatedCost(3, 3);
  double cost_1_1 = map_.getInflatedCost(1, 1);
  EXPECT_GT(cost_4_4, cost_3_3);   // (4,4) closer to (5,5) than (3,3)
  EXPECT_GT(cost_3_3, cost_1_1);   // (3,3) closer than (1,1)
  EXPECT_DOUBLE_EQ(cost_1_1, 0.0); // (1,1) >5 cells away
}

TEST_F(GridMapTest, IsPassable) {
  loadSimpleGrid();
  EXPECT_TRUE(map_.isPassable(0, 0));    // far from obstacle → free
  EXPECT_FALSE(map_.isPassable(5, 5));   // occupied → lethal
  // (4,4) is 1 cell from obstacle: cost = 254*(1-1/5)^2 = 162 > 100 → not passable
  EXPECT_FALSE(map_.isPassable(4, 4));
  // (2,2) is 3 cells from obstacle: cost = 254*(1-3/5)^2 = 40.6 < 100 → passable
  EXPECT_TRUE(map_.isPassable(2, 2));
}

TEST_F(GridMapTest, InflationEmptyMap) {
  std::vector<int8_t> data(100, 0);
  map_.loadFromRaw(data, 10, 10, 0.05, 0, 0);
  // All cells should be 0 cost.
  for (int r = 0; r < 10; ++r) {
    for (int c = 0; c < 10; ++c) {
      EXPECT_DOUBLE_EQ(map_.getInflatedCost(r, c), 0.0);
    }
  }
}

TEST_F(GridMapTest, OutOfBoundsReturnsLethal) {
  loadSimpleGrid();
  EXPECT_DOUBLE_EQ(map_.getInflatedCost(-1, -1), 254.0);
  EXPECT_DOUBLE_EQ(map_.getInflatedCost(100, 100), 254.0);
}

// ═══════════════════════════════════════════════════════════════
// Edge Cases
// ═══════════════════════════════════════════════════════════════

TEST_F(GridMapTest, SingleCellMap) {
  std::vector<int8_t> data = {0};
  map_.loadFromRaw(data, 1, 1, 0.05, 0, 0);
  EXPECT_EQ(map_.width(), 1);
  EXPECT_EQ(map_.height(), 1);
  EXPECT_TRUE(map_.isFree(0, 0));
}

TEST_F(GridMapTest, AllOccupiedMap) {
  std::vector<int8_t> data(25, 100);
  map_.loadFromRaw(data, 5, 5, 0.1, 0, 0);
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      EXPECT_TRUE(map_.isOccupied(r, c));
    }
  }
}

TEST_F(GridMapTest, AllUnknownMap) {
  std::vector<int8_t> data(25, -1);
  map_.loadFromRaw(data, 5, 5, 0.1, 0, 0);
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      EXPECT_TRUE(map_.isUnknown(r, c));
    }
  }
}

// ═══════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
