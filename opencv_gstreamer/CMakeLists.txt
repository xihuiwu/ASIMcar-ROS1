cmake_minimum_required(VERSION 2.8) 
PROJECT (camera)

find_package(OpenCV REQUIRED )

add_executable(camera camera.cpp)
target_link_libraries(camera ${OpenCV_LIBS})
