cmake_minimum_required(VERSION 2.8.3)
project(ydlidar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

set(HEADER_PATH "./include/")
FILE(GLOB HEADER_SRC 
  "${HEADER_PATH}/*.h"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(ydlidar_node src/ydlidar_node.cpp  ${HEADER_SRC})

target_link_libraries(ydlidar_node
   ${catkin_LIBRARIES} 
 )
