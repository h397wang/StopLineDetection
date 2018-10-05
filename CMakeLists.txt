cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "Wall")
project( StopLineDetection )
find_package( OpenCV REQUIRED )
add_executable( StopLineDetection
	src/stopline-detection.cpp
	src/helper.cpp )
target_link_libraries( StopLineDetection ${OpenCV_LIBS} )