cmake_minimum_required(VERSION 2.8.3)
project(front_radar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

catkin_install_python(PROGRAMS
   src/radar.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
