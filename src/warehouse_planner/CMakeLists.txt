cmake_minimum_required(VERSION 3.0.2)
project(warehouse_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  warehouse_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_planner
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    warehouse_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---- AStarPlanner library (pure algorithm, no ROS) ----
add_library(warehouse_planner
  src/astar_planner.cpp
)

target_link_libraries(warehouse_planner
  ${catkin_LIBRARIES}
)

# ---- ROS node ----
add_executable(astar_planner_node
  src/astar_planner_node.cpp
)

target_link_libraries(astar_planner_node
  warehouse_planner
  ${catkin_LIBRARIES}
)

# ---- Install ----
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS warehouse_planner astar_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ---- Unit tests ----
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_astar
    test/test_astar.cpp
    src/astar_planner.cpp
  )
  target_include_directories(test_astar PRIVATE include)
  target_link_libraries(test_astar
    ${catkin_LIBRARIES}
  )
endif()
