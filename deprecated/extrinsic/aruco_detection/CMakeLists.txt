cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	#INCLUDE_DIRS include
	#TODO Should the circle detector be placed in its own pkg?
	#LIBRARIES circle_detector
)

#add_library(circle_detector src/circle_detector.cpp)  
#target_link_libraries(circle_detector ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(aruco_detector src/detector.cpp)
target_link_libraries(aruco_detector ${OpenCV_LIBS} ${catkin_LIBRARIES})
