cmake_minimum_required(VERSION 2.8.3)
project(asimcar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  joy
  rosbag
  rostopic
  sensor_msgs
  std_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/joy_teleop_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
