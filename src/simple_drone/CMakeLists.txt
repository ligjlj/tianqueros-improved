cmake_minimum_required(VERSION 3.0.2)
project(simple_drone)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  gazebo_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/drone_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models/ worlds/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
