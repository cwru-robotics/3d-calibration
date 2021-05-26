cmake_minimum_required(VERSION 2.8.3)
project(sprayhead_calibration)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	message_generation
	#xform_utils
	#trajectory_utils
	#stella_jsp_utils
)

include_directories(${catkin_INCLUDE_DIRS})

add_service_files(FILES Sprayhead_Calibrate.srv)
generate_messages()

catkin_package(
	#DEPENDS xform_utils
	CATKIN_DEPENDS message_runtime
)

add_executable(sprayhead_cal_srv src/detector.cpp)
target_link_libraries(sprayhead_cal_srv ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(sprayhead_cal_srv ${${PROJECT_NAME}_EXPORTED_TARGETS})
