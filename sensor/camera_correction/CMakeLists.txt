cmake_minimum_required(VERSION 2.8.3)
project(camera_correction)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  cv_bridge
)

add_executable(correction_node src/correction_node.cpp
                           src/camera_correction.cpp)
add_dependencies(correction_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(correction_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS correction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
