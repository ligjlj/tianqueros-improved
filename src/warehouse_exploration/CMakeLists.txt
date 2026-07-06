cmake_minimum_required(VERSION 3.0.2)
project(warehouse_exploration)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs visualization_msgs std_msgs warehouse_utils)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_exploration
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs warehouse_utils)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(warehouse_exploration
  src/frontier_detector.cpp
  src/frontier_cluster.cpp
  src/goal_selector.cpp
  src/goal_manager.cpp
  src/reachability_checker.cpp
  src/exploration_fsm.cpp)
target_link_libraries(warehouse_exploration ${catkin_LIBRARIES})

add_executable(exploration_node src/exploration_node.cpp)
target_link_libraries(exploration_node warehouse_exploration ${catkin_LIBRARIES})

add_executable(coverage_monitor src/coverage_monitor.cpp)
target_link_libraries(coverage_monitor ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS warehouse_exploration exploration_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config/ launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frontier test/test_frontier.cpp
    src/frontier_detector.cpp src/frontier_cluster.cpp)
  target_include_directories(test_frontier PRIVATE include)
  target_link_libraries(test_frontier ${catkin_LIBRARIES})

  catkin_add_gtest(test_goal test/test_goal.cpp
    src/frontier_detector.cpp src/frontier_cluster.cpp src/goal_selector.cpp)
  target_include_directories(test_goal PRIVATE include)
  target_link_libraries(test_goal ${catkin_LIBRARIES})

  catkin_add_gtest(test_reachability test/test_reachability.cpp
    src/reachability_checker.cpp)
  target_include_directories(test_reachability PRIVATE include)
  target_link_libraries(test_reachability ${catkin_LIBRARIES})

  catkin_add_gtest(test_goal_manager test/test_goal_manager.cpp
    src/goal_manager.cpp)
  target_include_directories(test_goal_manager PRIVATE include)
  target_link_libraries(test_goal_manager ${catkin_LIBRARIES})
endif()
