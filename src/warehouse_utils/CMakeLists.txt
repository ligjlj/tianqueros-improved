cmake_minimum_required(VERSION 3.0.2)
project(warehouse_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_utils
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    std_msgs
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# ---- GridMap library ----
add_library(warehouse_utils
  src/grid_map.cpp
)

target_link_libraries(warehouse_utils
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

# ---- Install ----
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS warehouse_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ---- Unit tests (Phase 2) ----
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_grid_map
    test/test_grid_map.cpp
    src/grid_map.cpp
  )
  target_include_directories(test_grid_map PRIVATE include)
  target_link_libraries(test_grid_map
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
  )
endif()
