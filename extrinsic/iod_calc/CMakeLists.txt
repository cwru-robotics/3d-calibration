cmake_minimum_required(VERSION 3.0.2)
project(iod_calc)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(interocular_calculator src/interocular.cpp)
target_link_libraries(interocular_calculator
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	yaml-cpp
)
