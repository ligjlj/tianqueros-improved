/// Unit tests for RawCostmap and InflatedCostmap.

#include <gtest/gtest.h>
#include "warehouse_costmap/costmap_layer.hpp"

using namespace warehouse_costmap;

class CostmapTest : public ::testing::Test {
protected:
  RawCostmap raw_;
};

TEST_F(CostmapTest, RawLoadMsg) {
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 10; msg.info.height = 10; msg.info.resolution = 0.05;
  msg.info.origin.position.x = 0; msg.info.origin.position.y = 0;
  msg.data.resize(100, 0);
  msg.data[5*10+5] = 100;  // obstacle
  msg.data[9*10+9] = -1;   // unknown
  raw_.loadFromMsg(msg);

  EXPECT_EQ(raw_.width(), 10);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,0), 0);     // free
  EXPECT_DOUBLE_EQ(raw_.getCost(5,5), 254);   // occupied
  EXPECT_DOUBLE_EQ(raw_.getCost(9,9), -1);    // unknown
  EXPECT_DOUBLE_EQ(raw_.getCost(-1,0), 254);  // out of bounds
}

TEST_F(CostmapTest, RawThreshold) {
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 5; msg.info.height = 1; msg.info.resolution = 0.05;
  msg.data = {0, 49, 50, 51, 100};
  raw_.loadFromMsg(msg);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,0), 0);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,1), 0);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,2), 0);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,3), 254);
  EXPECT_DOUBLE_EQ(raw_.getCost(0,4), 254);
}

TEST_F(CostmapTest, InflateGradient) {
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 20; msg.info.height = 20; msg.info.resolution = 0.05;
  msg.data.resize(400, 0);
  msg.data[10*20+10] = 100;  // single obstacle
  raw_.loadFromMsg(msg);

  InflatedCostmap inf;
  inf.buildFrom(raw_, 0.3);  // 6 cells

  EXPECT_GT(inf.getCost(10,10), 250);  // obstacle center = lethal
  EXPECT_NEAR(inf.getCost(0,0), 0, 1); // far away = free
  double c1 = inf.getCost(8,10);  // 2 cells away
  double c2 = inf.getCost(5,10);  // 5 cells away
  EXPECT_GT(c1, c2);  // closer = higher cost
}

TEST_F(CostmapTest, InflateEmpty) {
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 10; msg.info.height = 10; msg.info.resolution = 0.05;
  msg.data.resize(100, 0);
  raw_.loadFromMsg(msg);

  InflatedCostmap inf;
  inf.buildFrom(raw_, 0.3);
  for (int r=0;r<10;++r) for (int c=0;c<10;++c)
    EXPECT_DOUBLE_EQ(inf.getCost(r,c), 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
