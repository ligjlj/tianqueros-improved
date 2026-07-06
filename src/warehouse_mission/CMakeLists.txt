cmake_minimum_required(VERSION 3.0.2)
project(warehouse_mission)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs geometry_msgs warehouse_utils)
catkin_package(CATKIN_DEPENDS roscpp nav_msgs std_msgs geometry_msgs warehouse_utils)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mission_node src/mission_node.cpp)
target_link_libraries(mission_node ${catkin_LIBRARIES})

install(TARGETS mission_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
