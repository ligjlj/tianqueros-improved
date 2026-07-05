cmake_minimum_required(VERSION 3.0.2)
project(warehouse_motion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motion_controller_node src/motion_controller_node.cpp)
target_link_libraries(motion_controller_node ${catkin_LIBRARIES})

install(TARGETS motion_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
