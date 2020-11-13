cmake_minimum_required(VERSION 2.8.3)
project(intrinsic_calibration)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cc_utils
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
)

add_executable(int_calibrator src/calibrator.cpp)
target_link_libraries(int_calibrator ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
