cmake_minimum_required(VERSION 2.8.3)
project(imu_ekf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  tf
  tf2_ros
  rosbag
  rostopic
  rviz
  sensor_msgs
  std_msgs
#  DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

