cmake_minimum_required(VERSION 2.8.3)
project(intrinsic_acquisition)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(intrinsic_acquisition src/intrinsic_acquisition.cpp)
target_link_libraries(intrinsic_acquisition ${catkin_LIBRARIES} yaml-cpp)

add_executable(intrinsic_axes_helper src/intrinsic_axes_helper.cpp)
target_link_libraries(intrinsic_axes_helper ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)
