#include "warehouse_exploration/coverage_monitor.hpp"
int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_monitor");
  warehouse_exploration::CoverageMonitor cm;
  cm.spin();
  return 0;
}
