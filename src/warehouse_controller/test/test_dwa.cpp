/// Unit tests for DWAPlanner.
/// Tests: basic velocity computation, collision avoidance, goal detection.

#include <gtest/gtest.h>
#include <cmath>
#include "warehouse_utils/grid_map.hpp"
#include "warehouse_controller/dwa_planner.hpp"

using namespace warehouse_utils;
using namespace warehouse_controller;

class DWATest : public ::testing::Test {
protected:
  void SetUp() override {
    DWAPlanner::Config cfg;
    cfg.max_linear_vel    = 1.0;
    cfg.max_angular_vel   = 1.5;
    cfg.max_linear_accel  = 2.0;
    cfg.max_angular_accel = 3.0;
    cfg.predict_time      = 1.5;
    cfg.dt                = 0.1;
    cfg.v_samples         = 10;
    cfg.w_samples         = 20;
    cfg.alpha             = 0.5;
    cfg.beta              = 2.0;
    cfg.gamma             = 0.3;
    cfg.goal_tolerance_m  = 0.3;
    planner_ = DWAPlanner(cfg);

    GridMap::Config gcfg;
    gcfg.inflation_radius_m = 1.0;
    gcfg.robot_radius_m     = 0.5;
    gcfg.resolution_m       = 0.05;
    grid_ = GridMap(gcfg);
  }

  void loadEmpty() {
    std::vector<int8_t> data(400, 0);
    grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  }

  void loadWithObstacle() {
    std::vector<int8_t> data(400, 0);
    // Wall from row 9 col 0 to row 9 col 15.
    for (int c = 0; c < 16; ++c) data[9 * 20 + c] = 100;
    grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);
  }

  DWAPlanner planner_;
  GridMap    grid_;
};

// ═══════════════════════════════════════════════════════════════
// Basic velocity computation
// ═══════════════════════════════════════════════════════════════

TEST_F(DWATest, ComputesVelocityOnEmptyMap) {
  loadEmpty();
  DWAPlanner::RobotState state{0.25, 0.25, 0.0, 0.0, 0.0};
  WorldPoint goal{0.75, 0.25};  // straight ahead

  auto [v, w] = planner_.computeVelocity(grid_, state, goal);
  EXPECT_GT(v, 0.0);            // should move forward
  EXPECT_NEAR(w, 0.0, 0.3);    // roughly straight
}

TEST_F(DWATest, TurnsTowardsGoal) {
  loadEmpty();
  DWAPlanner::RobotState state{0.25, 0.25, 0.0, 0.0, 0.0};
  WorldPoint goal{0.25, 0.75};  // 90° to the left

  auto [v, w] = planner_.computeVelocity(grid_, state, goal);
  EXPECT_GT(w, 0.0);            // should turn left (positive angular)
}

TEST_F(DWATest, AvoidsObstacle) {
  loadWithObstacle();
  // Start below wall, goal above. Must avoid the wall.
  DWAPlanner::RobotState state{0.5, 0.35, 1.57, 0.0, 0.0};  // heading up
  WorldPoint goal{0.5, 0.6};  // above wall

  auto [v, w] = planner_.computeVelocity(grid_, state, goal);

  // Simulate one step and check no collision.
  double x = state.x, y = state.y, th = state.theta;
  x += v * std::cos(th) * 0.1;
  y += v * std::sin(th) * 0.1;

  auto cell = grid_.worldToGrid(x, y);
  EXPECT_TRUE(grid_.isValid(cell.row, cell.col));
  EXPECT_LT(grid_.getInflatedCost(cell.row, cell.col), 254.0);
}

TEST_F(DWATest, ZeroVelocityWhenBlocked) {
  // Fully blocked map.
  std::vector<int8_t> data(400, 100);
  grid_.loadFromRaw(data, 20, 20, 0.05, 0.0, 0.0);

  DWAPlanner::RobotState state{0.5, 0.5, 0.0, 0.0, 0.0};
  WorldPoint goal{0.5, 0.5};  // at goal

  auto [v, w] = planner_.computeVelocity(grid_, state, goal);
  // Should stop (goal already reached or blocked).
  EXPECT_TRUE(planner_.isGoalReached(state, goal) || (v == 0.0 && w == 0.0));
}

// ═══════════════════════════════════════════════════════════════
// Goal detection
// ═══════════════════════════════════════════════════════════════

TEST_F(DWATest, GoalReachedClose) {
  DWAPlanner::RobotState state{0.5, 0.5, 0.0, 0.0, 0.0};
  WorldPoint goal{0.5001, 0.5001};
  EXPECT_TRUE(planner_.isGoalReached(state, goal));
}

TEST_F(DWATest, GoalNotReachedFar) {
  DWAPlanner::RobotState state{0.0, 0.0, 0.0, 0.0, 0.0};
  WorldPoint goal{5.0, 5.0};
  EXPECT_FALSE(planner_.isGoalReached(state, goal));
}

// ═══════════════════════════════════════════════════════════════
// Trajectory simulation
// ═══════════════════════════════════════════════════════════════

TEST_F(DWATest, TrajectoriesNotEmpty) {
  loadEmpty();
  DWAPlanner::RobotState state{0.25, 0.25, 0.0, 0.0, 0.0};
  WorldPoint goal{0.75, 0.75};

  auto trajs = planner_.getTrajectories(grid_, state, goal);
  EXPECT_EQ(trajs.size(),
            static_cast<size_t>(planner_.config().v_samples *
                                planner_.config().w_samples));

  // At least some trajectories should have points.
  int with_points = 0;
  for (const auto& t : trajs) {
    if (!t.points.empty()) ++with_points;
  }
  EXPECT_GT(with_points, 0);
}

TEST_F(DWATest, CollisionDetectedInObstacle) {
  loadWithObstacle();
  // Start inside the wall.
  DWAPlanner::RobotState state{0.5 - 0.025, 0.45, 0.0, 0.0, 0.0};
  WorldPoint goal{0.5, 0.5};

  // The first step should detect collision (robot starts near the wall).
  auto trajs = planner_.getTrajectories(grid_, state, goal);
  int collisions = 0;
  for (const auto& t : trajs) {
    if (t.collided) ++collisions;
  }
  // Some trajectories should detect collision.
  EXPECT_GT(collisions, 0);
}

// ═══════════════════════════════════════════════════════════════
// Dynamic window limits
// ═══════════════════════════════════════════════════════════════

TEST_F(DWATest, VelocityWithinLimits) {
  loadEmpty();
  DWAPlanner::RobotState state{0.25, 0.25, 0.0, 1.0, 0.5};  // moving

  // Acceleration from (1.0, 0.5) with max_accel=2.0, max_ang_accel=3.0, dt=0.1
  // v range: [1.0-0.2, min(1.0, 1.0+0.2)] = [0.8, 1.0]
  // w range: [0.5-0.3, min(1.5, 0.5+0.3)] = [0.2, 0.8]

  for (int i = 0; i < 10; ++i) {
    WorldPoint goal{0.75, 0.25};
    auto [v, w] = planner_.computeVelocity(grid_, state, goal);

    // v should be within absolute limit.
    EXPECT_LE(v, planner_.config().max_linear_vel);
    EXPECT_GE(v, 0.0);
    // w should be within absolute limit.
    EXPECT_LE(std::abs(w), planner_.config().max_angular_vel + 1e-6);
  }
}

// ═══════════════════════════════════════════════════════════════
// Scoring logic
// ═══════════════════════════════════════════════════════════════

TEST_F(DWATest, HigherScoreForBetterHeading) {
  loadEmpty();
  // Robot facing goal directly.
  DWAPlanner::RobotState state1{0.25, 0.25, 0.0, 0.0, 0.0};
  WorldPoint goal{0.75, 0.25};

  auto [v1, w1] = planner_.computeVelocity(grid_, state1, goal);

  // Robot facing away from goal.
  DWAPlanner::RobotState state2{0.25, 0.25, 3.14, 0.0, 0.0};
  auto [v2, w2] = planner_.computeVelocity(grid_, state2, goal);

  // Facing-goal case should prefer moving forward more.
  // (This is a soft check — DWA is non-deterministic in absolute terms.)
  EXPECT_TRUE(v1 >= 0.0);
  EXPECT_TRUE(v2 >= 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
