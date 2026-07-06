/// Unit tests for GoalManager state machine.
#include <gtest/gtest.h>
#include "warehouse_exploration/goal_manager.hpp"

using namespace warehouse_exploration;
using warehouse_utils::WorldPoint;

class GoalManagerTest : public ::testing::Test {
protected:
  GoalManager gm_;
};

TEST_F(GoalManagerTest, AddCandidateDedup) {
  gm_.addCandidate({1,1});
  gm_.addCandidate({1.01, 1.01});
  EXPECT_EQ(gm_.size(), 1u);
}

TEST_F(GoalManagerTest, StateProgression) {
  gm_.addCandidate({2,2});
  EXPECT_EQ(gm_.getState({2,2}), GoalState::UNTRIED);

  gm_.markTried({2,2});
  EXPECT_EQ(gm_.getState({2,2}), GoalState::TRIED);

  gm_.markFailed({2,2});
  EXPECT_EQ(gm_.getState({2,2}), GoalState::FAILED);
  EXPECT_EQ(gm_.getFailCount({2,2}), 1);
}

TEST_F(GoalManagerTest, BlacklistAfter3Fails) {
  gm_.addCandidate({3,3});
  gm_.markFailed({3,3}); gm_.markFailed({3,3}); gm_.markFailed({3,3});
  EXPECT_EQ(gm_.getState({3,3}), GoalState::BLACKLISTED);
  EXPECT_TRUE(gm_.isBlacklisted({3,3}));
}

TEST_F(GoalManagerTest, MarkCompletedResets) {
  gm_.addCandidate({4,4});
  gm_.markFailed({4,4}); gm_.markFailed({4,4});
  gm_.markCompleted({4,4});
  EXPECT_EQ(gm_.getState({4,4}), GoalState::UNTRIED);
  EXPECT_EQ(gm_.getFailCount({4,4}), 0);
}

int main(int argc, char** argv) {
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
