cmake_minimum_required(VERSION 3.0.2)
project(intrinsic_simulation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(sim_mover src/sim_target_positioner.cpp)
target_link_libraries(sim_mover ${catkin_LIBRARIES})
