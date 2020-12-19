cmake_minimum_required(VERSION 2.8.3)
project(lidar_aruco)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	cc_utils
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
	#INCLUDE_DIRS include
	#TODO Should the circle detector be placed in its own pkg?
	#LIBRARIES circle_detector
)

#add_library(circle_detector src/circle_detector.cpp)  
#target_link_libraries(circle_detector ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(lidar_aruco src/detector.cpp)
target_link_libraries(lidar_aruco ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)
