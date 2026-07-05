/// Unit tests for PathTracker (Pure Pursuit).

#include <gtest/gtest.h>
#include <cmath>
#include "warehouse_controller/path_tracker.hpp"

using namespace warehouse_utils;
using namespace warehouse_controller;

class TrackerTest : public ::testing::Test {
protected:
  void SetUp() override {
    PathTracker::Config cfg;
    cfg.lookahead_distance_m  = 0.5;
    cfg.path_error_threshold_m = 0.1;
    tracker_ = PathTracker(cfg);
  }

  PathTracker tracker_;
};

TEST_F(TrackerTest, EmptyPathReturnsInvalid) {
  auto result = tracker_.track({}, WorldPoint{0, 0});
  EXPECT_FALSE(result.valid);
}

TEST_F(TrackerTest, SinglePointPath) {
  std::vector<WorldPoint> path = {{1.0, 2.0}};
  auto result = tracker_.track(path, WorldPoint{0, 0});
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.lookahead_point.x, 1.0);
  EXPECT_DOUBLE_EQ(result.lookahead_point.y, 2.0);
}

TEST_F(TrackerTest, StraightLineLookahead) {
  // Path: (0,0) → (2,0), robot at (0,0)
  std::vector<WorldPoint> path = {{0, 0}, {0.25, 0}, {0.5, 0}, {0.75, 0}, {1.0, 0}, {2.0, 0}};
  auto result = tracker_.track(path, WorldPoint{0, 0});

  EXPECT_TRUE(result.valid);
  // Lookahead 0.5m from (0,0) along straight line → (0.5, 0)
  EXPECT_NEAR(result.lookahead_point.x, 0.5, 0.05);
  EXPECT_NEAR(result.lookahead_point.y, 0.0, 0.01);
  EXPECT_EQ(result.closest_index, 0);
}

TEST_F(TrackerTest, RobotMidPath) {
  // Robot at (1.0, 0) along straight path.
  std::vector<WorldPoint> path = {{0, 0}, {0.5, 0}, {1.0, 0}, {1.5, 0}, {2.0, 0}};
  auto result = tracker_.track(path, WorldPoint{1.0, 0});

  EXPECT_TRUE(result.valid);
  // Closest is (1.0, 0). Lookahead 0.5m → (1.5, 0).
  EXPECT_NEAR(result.lookahead_point.x, 1.5, 0.05);
  EXPECT_NEAR(result.cross_track_error, 0.0, 0.01);
}

TEST_F(TrackerTest, RobotOffPath) {
  // Robot is 0.3m to the side of the path.
  std::vector<WorldPoint> path = {{0, 0}, {1, 0}, {2, 0}};
  auto result = tracker_.track(path, WorldPoint{1.0, 0.3});

  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.cross_track_error, 0.3, 0.01);
}

TEST_F(TrackerTest, LookaheadExceedsPath) {
  // Short path, lookahead extends beyond the end.
  std::vector<WorldPoint> path = {{0, 0}, {0.1, 0}};
  auto result = tracker_.track(path, WorldPoint{0, 0});

  EXPECT_TRUE(result.valid);
  // Should return the last point on the path.
  EXPECT_NEAR(result.lookahead_point.x, 0.1, 0.01);
}

TEST_F(TrackerTest, CrossTrackErrorCalculation) {
  // The tracker computes distance to the closest discrete path point,
  // not the perpendicular distance to the continuous line.
  std::vector<WorldPoint> path = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  auto result = tracker_.track(path, WorldPoint{1.5, 0.05});

  EXPECT_TRUE(result.valid);
  // Closest discrete point is (1,0) or (2,0) at distance ~0.5025.
  EXPECT_GT(result.cross_track_error, 0.4);
  EXPECT_LT(result.cross_track_error, 0.6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
