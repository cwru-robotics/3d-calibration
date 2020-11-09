cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_toplevel)

#find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)

#find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	geometry_msgs
	#cv_bridge
	#xform_utils
	#trajectory_utils
	#stella_jsp_utils
)

add_service_files(FILES RunCalibration.srv)
generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${EIGEN3_INCLUDE_DIR})


catkin_package(
	CATKIN_DEPENDS message_runtime
	#DEPENDS message_runtime system_lib
	#INCLUDE_DIRS include
	#CATKIN_DEPENDS xform_utils
)

add_executable(extrinsic_exec_service src/extrinsic_exec.cpp)
#This next line is necessary for referenceing message defs in the same package as source code: https://answers.ros.org/question/53265/catkin-messages-and-node-in-same-package/
add_dependencies(extrinsic_exec_service extrinsic_toplevel_gencpp)
target_link_libraries(extrinsic_exec_service ${catkin_LIBRARIES})# yaml-cpp)
