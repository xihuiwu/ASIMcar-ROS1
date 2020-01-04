cmake_minimum_required(VERSION 2.8.3)
project(front_radar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

find_package(OpenCV 3.4.5 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)
