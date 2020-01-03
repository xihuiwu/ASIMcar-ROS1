cmake_minimum_required(VERSION 2.8.3)
project(front_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
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
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})
