cmake_minimum_required(VERSION 2.8.3)
project(single_corner)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES single_corner
)

add_library(single_corner src/single_corner.cpp)  
target_link_libraries(single_corner 
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)

add_executable(test_single_corner src/test_single_corner.cpp)
target_link_libraries(test_single_corner
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	single_corner
)
