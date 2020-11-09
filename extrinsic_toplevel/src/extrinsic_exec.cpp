#include <ros/ros.h>
#include <extrinsic_toplevel/RunCalibration.h>

bool CB_cal_service(
	extrinsic_toplevel::RunCalibration::Request& request,
	extrinsic_toplevel::RunCalibration::Response& response
){
	system("roslaunch extrinsic_acquisition extrinsic_acquire.launch");
	system("roslaunch aruco_detection aruco_detect.launch");
	system("roslaunch extrinsic_calibration calibrate.launch");

	return true;
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "one_click_extrinsic");
	ros::NodeHandle nh;
	
	ros::ServiceServer cal = nh.advertiseService("/calibrate_extrinsic", CB_cal_service);
	
	ros::spin();

	return 0;
}
