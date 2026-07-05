/// Unit tests for AStarPlanner.
/// Tests: trivial paths, blocked scenarios, 100 random start/goal pairs, statistics.

#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <algorithm>

#include <ros/console.h>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_planner/astar_planner.hpp"

using namespace warehouse_utils;
using namespace warehouse_planner;

// ═══════════════════════════════════════════════════════════════
// Fixture: 20x20 empty grid, 1.0m inflation, 0.5m robot radius
// ═══════════════════════════════════════════════════════════════

class AStarTest : public ::testing::Test {
protected:
  void SetUp() override {
    AStarPlanner::Config cfg;
    cfg.heuristic_weight   = 1.0;
    cfg.collision_margin_m = 0.05;
    planner_ = AStarPlanner(cfg);

    GridMap::Config gcfg;
    gcfg.inflation_radius_m = 1.0;
    gcfg.robot_radius_m     = 0.5;
    gcfg.resolution_m       = 0.05;
    grid_ = GridMap(gcfg);
  }

  /// Load an empty 20x20 grid.
  void loadEmpty() {
    std::vector<int8_t> data(400, 0);
    grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  }

  /// Load a grid with a horizontal wall at row 10, gap at col 10.
  void loadWallWithGap() {
    std::vector<int8_t> data(400, 0);
    // Wall across row 10.
    for (int c = 0; c < 20; ++c) {
      if (c != 10) {  // gap at col 10
        data[10 * 20 + c] = 100;
      }
    }
    grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  }

  AStarPlanner planner_;
  GridMap      grid_;
};

// ═══════════════════════════════════════════════════════════════
// Basic Paths
// ═══════════════════════════════════════════════════════════════

TEST_F(AStarTest, StraightLinePath) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},   // (5,5)
                              WorldPoint{0.75, 0.25});  // (15,5)
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 2u);
  EXPECT_GT(result.path_length_m, 0.4);   // ~0.5m
  EXPECT_LT(result.path_length_m, 0.8);
  EXPECT_GT(result.nodes_expanded, 0);
}

TEST_F(AStarTest, DiagonalPath) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},   // (5,5)
                              WorldPoint{0.75, 0.75});  // (15,15)
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 2u);
}

TEST_F(AStarTest, SameCellNoOp) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},
                              WorldPoint{0.25, 0.25});
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 1u);
  EXPECT_NEAR(result.path_length_m, 0.0, 1e-6);
}

TEST_F(AStarTest, WallWithGapFindsPath) {
  loadWallWithGap();
  // Start above wall, goal below. Must go through gap at col 10.
  auto result = planner_.plan(grid_,
                              WorldPoint{0.5, 0.25},    // top (row 5, col 10)
                              WorldPoint{0.5, 0.75});   // bottom (row 15, col 10)
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 5u);
}

TEST_F(AStarTest, WallNoGapNoPath) {
  // Full wall, no gap.
  std::vector<int8_t> data(400, 0);
  for (int c = 0; c < 20; ++c) {
    data[10 * 20 + c] = 100;
  }
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  auto result = planner_.plan(grid_,
                              WorldPoint{0.5, 0.25},    // top
                              WorldPoint{0.5, 0.75});   // bottom
  EXPECT_FALSE(result.success);
  EXPECT_GT(result.nodes_expanded, 0);
}

TEST_F(AStarTest, StartBlocked) {
  loadEmpty();
  // Place start inside an occupied cell.
  std::vector<int8_t> data(400, 0);
  data[5 * 20 + 5] = 100;
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},   // blocked
                              WorldPoint{0.75, 0.75});
  EXPECT_FALSE(result.success);
}

TEST_F(AStarTest, GoalBlocked) {
  loadEmpty();
  std::vector<int8_t> data(400, 0);
  data[15 * 20 + 15] = 100;
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},
                              WorldPoint{0.75, 0.75});   // blocked
  EXPECT_FALSE(result.success);
}

TEST_F(AStarTest, OutOfBoundsStart) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{-10.0, -10.0},
                              WorldPoint{0.5, 0.5});
  EXPECT_FALSE(result.success);
}

TEST_F(AStarTest, OutOfBoundsGoal) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{0.5, 0.5},
                              WorldPoint{100.0, 100.0});
  EXPECT_FALSE(result.success);
}

// ═══════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════

TEST_F(AStarTest, ResultStatisticsPopulated) {
  loadEmpty();
  auto result = planner_.plan(grid_,
                              WorldPoint{0.25, 0.25},
                              WorldPoint{0.75, 0.75});
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.nodes_expanded, 0);
  EXPECT_GT(result.path_length_m, 0.0);
  EXPECT_GT(result.planning_time_ms, 0.0);
}

// ═══════════════════════════════════════════════════════════════
// 100 Random Start/Goal Pairs
// ═══════════════════════════════════════════════════════════════

TEST_F(AStarTest, Random100Pairs) {
  loadEmpty();
  std::srand(42);  // deterministic

  int successes   = 0;
  int failures    = 0;
  double total_len = 0.0;

  for (int i = 0; i < 100; ++i) {
    const int sr = std::rand() % 20;
    const int sc = std::rand() % 20;
    const int gr = std::rand() % 20;
    const int gc = std::rand() % 20;

    // Skip if start == goal.
    if (sr == gr && sc == gc) continue;

    WorldPoint start = grid_.gridToWorld(sr, sc);
    WorldPoint goal  = grid_.gridToWorld(gr, gc);

    auto result = planner_.plan(grid_, start, goal);

    if (result.success) {
      ++successes;
      total_len += result.path_length_m;

      // Path should start near start and end near goal.
      EXPECT_NEAR(result.path.front().x, start.x, 0.1);
      EXPECT_NEAR(result.path.front().y, start.y, 0.1);
      EXPECT_NEAR(result.path.back().x,  goal.x,  0.1);
      EXPECT_NEAR(result.path.back().y,  goal.y,  0.1);

      // On empty grid, path should be roughly monotonic.
      EXPECT_GT(result.nodes_expanded, 0);
    } else {
      ++failures;
    }
  }

  // On empty 20x20 grid, ALL pairs should be reachable.
  EXPECT_EQ(failures, 0);
  EXPECT_GT(successes, 90);  // at least 90 (some skipped due to start==goal)
  EXPECT_GT(total_len, 0.0);

  ROS_INFO_STREAM("Random100: " << successes << " succeeded, "
                  << failures << " failed, avg length = "
                  << (successes > 0 ? total_len / successes : 0.0) << " m");
}

// ═══════════════════════════════════════════════════════════════
// Heuristic weight behavior
// ═══════════════════════════════════════════════════════════════

TEST_F(AStarTest, WeightedHeuristicStillFindsPath) {
  loadEmpty();

  AStarPlanner::Config cfg;
  cfg.heuristic_weight = 2.0;  // greedy
  AStarPlanner greedy(cfg);

  auto result = greedy.plan(grid_,
                            WorldPoint{0.25, 0.25},
                            WorldPoint{0.75, 0.75});
  EXPECT_TRUE(result.success);
  // Should still find a path, but may expand fewer nodes than optimal.
  EXPECT_GT(result.nodes_expanded, 0);
}

// ═══════════════════════════════════════════════════════════════
// Maze-like sparse obstacles
// ═══════════════════════════════════════════════════════════════

TEST_F(AStarTest, SparseObstacleMaze) {
  std::vector<int8_t> data(400, 0);

  // Scatter some obstacles.
  data[3 * 20 + 5]  = 100;
  data[3 * 20 + 6]  = 100;
  data[7 * 20 + 12] = 100;
  data[7 * 20 + 13] = 100;
  data[12 * 20 + 8] = 100;
  data[16 * 20 + 15] = 100;

  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  auto result = planner_.plan(grid_,
                              WorldPoint{0.1, 0.1},     // near origin
                              WorldPoint{0.9, 0.9});    // near opposite corner
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 5u);

  // Path should not pass through obstacle cells.
  for (const auto& wp : result.path) {
    auto cell = grid_.worldToGrid(wp.x, wp.y);
    EXPECT_FALSE(grid_.isOccupied(cell.row, cell.col));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
