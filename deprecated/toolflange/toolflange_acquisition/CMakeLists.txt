cmake_minimum_required(VERSION 2.8.3)
project(toolflange_acquisition)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	xform_utils
	trajectory_utils
	stella_jsp_utils
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
	DEPENDS xform_utils
	CATKIN_DEPENDS xform_utils
)

add_executable(toolflange_acquisition src/toolflange_acquisition.cpp)
target_link_libraries(toolflange_acquisition ${catkin_LIBRARIES} yaml-cpp)
