cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_acquisition)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
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

add_executable(extrinsic_acquisition src/extrinsic_acquisition.cpp)
target_link_libraries(extrinsic_acquisition ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)
