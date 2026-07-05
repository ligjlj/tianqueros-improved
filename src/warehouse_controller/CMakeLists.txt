cmake_minimum_required(VERSION 3.0.2)
project(warehouse_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs visualization_msgs tf2_ros warehouse_utils)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_controller
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs tf2_ros warehouse_utils)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(warehouse_controller src/dwa_planner.cpp src/path_tracker.cpp)
target_link_libraries(warehouse_controller ${catkin_LIBRARIES})

add_executable(dwa_controller_node src/dwa_controller_node.cpp)
target_link_libraries(dwa_controller_node warehouse_controller ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS warehouse_controller dwa_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dwa test/test_dwa.cpp src/dwa_planner.cpp)
  target_include_directories(test_dwa PRIVATE include)
  target_link_libraries(test_dwa ${catkin_LIBRARIES})

  catkin_add_gtest(test_tracker test/test_tracker.cpp src/path_tracker.cpp)
  target_include_directories(test_tracker PRIVATE include)
  target_link_libraries(test_tracker ${catkin_LIBRARIES})
endif()
