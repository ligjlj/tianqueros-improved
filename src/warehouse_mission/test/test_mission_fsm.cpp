/// Unit tests for MissionFSM state transitions.

#include <gtest/gtest.h>
#include "warehouse_mission/mission_fsm.hpp"

using namespace warehouse_mission;

class MissionFSMTest : public ::testing::Test {
protected:
  void SetUp() override { fsm_.reset(); }
  MissionFSM fsm_;
};

TEST_F(MissionFSMTest, StartsInBoot) {
  EXPECT_EQ(fsm_.state(), MissionFSM::BOOT);
}

TEST_F(MissionFSMTest, BootToWaitMap) {
  ros::WallDuration(1.1).sleep();
  fsm_.update();
  EXPECT_EQ(fsm_.state(), MissionFSM::WAIT_MAP);
}

TEST_F(MissionFSMTest, WaitMapStays) {
  ros::WallDuration(1.1).sleep(); fsm_.update();
  ASSERT_EQ(fsm_.state(), MissionFSM::WAIT_MAP);
  for (int i = 0; i < 10; ++i) { fsm_.update(); ros::WallDuration(0.01).sleep(); }
  EXPECT_EQ(fsm_.state(), MissionFSM::WAIT_MAP);
}

TEST_F(MissionFSMTest, MapReadyToScan) {
  ros::WallDuration(1.1).sleep(); fsm_.update();
  fsm_.onMapReady(); fsm_.update();
  EXPECT_EQ(fsm_.state(), MissionFSM::INITIAL_SCAN);
}

TEST_F(MissionFSMTest, ScanToExplore) {
  ros::WallDuration(1.1).sleep(); fsm_.update();
  fsm_.onMapReady(); fsm_.update();
  // Simulate 360° rotation via incremental yaw updates (avoid wrap-around issue).
  for (double y = 0.5; y <= 6.0; y += 0.5) { fsm_.onYawUpdate(y); }
  ros::WallDuration(8.1).sleep(); fsm_.update();
  EXPECT_EQ(fsm_.state(), MissionFSM::EXPLORATION);
}

TEST_F(MissionFSMTest, ExploreStays) {
  ros::WallDuration(1.1).sleep(); fsm_.update();
  fsm_.onMapReady(); fsm_.update();
  for (double y = 0.5; y <= 6.0; y += 0.5) { fsm_.onYawUpdate(y); }
  ros::WallDuration(8.1).sleep(); fsm_.update();
  ASSERT_EQ(fsm_.state(), MissionFSM::EXPLORATION);
  for (int i = 0; i < 10; ++i) { fsm_.update(); ros::WallDuration(0.01).sleep(); }
  EXPECT_EQ(fsm_.state(), MissionFSM::EXPLORATION);
}

TEST_F(MissionFSMTest, DoneToFinish) {
  ros::WallDuration(1.1).sleep(); fsm_.update();
  fsm_.onMapReady(); fsm_.update();
  for (double y = 0.5; y <= 6.0; y += 0.5) { fsm_.onYawUpdate(y); }
  ros::WallDuration(8.1).sleep(); fsm_.update();
  ASSERT_EQ(fsm_.state(), MissionFSM::EXPLORATION);
  fsm_.onExplorationDone(); fsm_.update();
  EXPECT_EQ(fsm_.state(), MissionFSM::FINISHED);
}

TEST_F(MissionFSMTest, CallbacksFire) {
  int count = 0; std::string last;
  fsm_.onStateChange([&](const std::string& s) { ++count; last = s; });
  ros::WallDuration(1.1).sleep(); fsm_.update();
  EXPECT_GE(count, 1);
  EXPECT_EQ(last, "WAIT_MAP");
}

TEST_F(MissionFSMTest, RotateOnOff) {
  bool r = false;
  fsm_.onNeedRotate([&](bool v) { r = v; });
  ros::WallDuration(1.1).sleep(); fsm_.update();
  fsm_.onMapReady(); fsm_.update();
  EXPECT_TRUE(r);   // rotating during scan
  for (double y = 0.5; y <= 6.0; y += 0.5) { fsm_.onYawUpdate(y); }
  ros::WallDuration(8.1).sleep(); fsm_.update();
  EXPECT_FALSE(r);  // stopped after scan
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
