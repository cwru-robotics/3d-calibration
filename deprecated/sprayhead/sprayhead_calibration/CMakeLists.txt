cmake_minimum_required(VERSION 3.0.2)
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

add_executable(sprayhead_hor_cal_srv src/detector_h.cpp)
target_link_libraries(sprayhead_hor_cal_srv ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(sprayhead_hor_cal_srv ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(sprayhead_ver_cal_srv src/detector_v.cpp)
target_link_libraries(sprayhead_ver_cal_srv ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(sprayhead_ver_cal_srv ${${PROJECT_NAME}_EXPORTED_TARGETS})
