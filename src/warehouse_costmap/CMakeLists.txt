cmake_minimum_required(VERSION 3.0.2)
project(warehouse_costmap)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs warehouse_utils)
catkin_package(INCLUDE_DIRS include LIBRARIES warehouse_costmap
  CATKIN_DEPENDS roscpp nav_msgs warehouse_utils)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(warehouse_costmap src/costmap_layer.cpp)
target_link_libraries(warehouse_costmap ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS warehouse_costmap ARCHIVE LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_costmap test/test_costmap.cpp src/costmap_layer.cpp)
  target_include_directories(test_costmap PRIVATE include)
  target_link_libraries(test_costmap ${catkin_LIBRARIES})
endif()
