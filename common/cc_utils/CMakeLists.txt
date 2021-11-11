cmake_minimum_required(VERSION 3.0.2)
project(cc_utils)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cc_utils
)

add_library(cc_utils src/cc_utils.cpp)
target_link_libraries(cc_utils
      ${catkin_LIBRARIES}
      #${OpenCV_LIBRARIES}
     # ${PCL_LIBRARIES}
)
