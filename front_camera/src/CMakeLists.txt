cmake_minimum_required(VERSION 2.8.3)
project(asimcar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3.4.5 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

add_executable(camera src/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})
