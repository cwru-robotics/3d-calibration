cmake_minimum_required(VERSION 3.0.2)
project(test_dewarp)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(test_dewarp_file src/test_dewarp_file.cpp)
target_link_libraries(test_dewarp_file ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)

add_executable(test_dewarp_topic src/test_dewarp_topic.cpp)
target_link_libraries(test_dewarp_topic ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)
