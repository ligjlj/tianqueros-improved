cmake_minimum_required(VERSION 3.0.2)
project(warehouse_mission)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs geometry_msgs warehouse_utils)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_mission
  CATKIN_DEPENDS roscpp nav_msgs std_msgs geometry_msgs warehouse_utils)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(warehouse_mission src/mission_fsm.cpp)
target_link_libraries(warehouse_mission ${catkin_LIBRARIES})

add_executable(mission_node src/mission_node.cpp)
target_link_libraries(mission_node warehouse_mission ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS warehouse_mission mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mission_fsm test/test_mission_fsm.cpp src/mission_fsm.cpp)
  target_include_directories(test_mission_fsm PRIVATE include)
  target_link_libraries(test_mission_fsm ${catkin_LIBRARIES})
endif()
