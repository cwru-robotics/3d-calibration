cmake_minimum_required(VERSION 3.0.2)
project(dotboard_detection)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	#TODO Should the circle detector be placed in its own pkg?
	LIBRARIES circle_detector
)

add_library(circle_detector_tes src/circle_detector.cpp)  
target_link_libraries(circle_detector_tes ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(dots_detector src/detector.cpp)
target_link_libraries(dots_detector
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	yaml-cpp
	circle_detector_tes
)
