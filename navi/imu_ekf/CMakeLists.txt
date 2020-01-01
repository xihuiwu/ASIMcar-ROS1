cmake_minimum_required(VERSION 2.8.3)
project(imu_ekf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  bicycle_msgs
  vesc_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  rosbag
  rostopic
  std_msgs
  sensor_msgs
  geometry_msgs
  bicycle_msgs
  vesc_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
