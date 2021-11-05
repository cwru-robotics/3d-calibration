cmake_minimum_required(VERSION 3.0.2)
project(stella_jsp_utils)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	trajectory_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES stella_jsp_utils
)

add_library(stella_jsp_utils src/stella_jsp_utils.cpp)
target_link_libraries(stella_jsp_utils ${catkin_LIBRARIES})

add_executable(go src/go.cpp)
target_link_libraries(go stella_jsp_utils ${catkin_LIBRARIES})
