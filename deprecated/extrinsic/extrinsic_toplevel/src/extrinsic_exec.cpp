#include <ros/ros.h>
#include <ros/package.h>
#include <extrinsic_toplevel/RunCalibration.h>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

bool CB_cal_service(
	extrinsic_toplevel::RunCalibration::Request& request,
	extrinsic_toplevel::RunCalibration::Response& response
){
	system("roslaunch extrinsic_acquisition extrinsic_acquire.launch");
	system("roslaunch aruco_detection aruco_detect.launch");
	system("roslaunch extrinsic_calibration calibrate.launch");
	
	YAML::Node intrinsic_file;
	std::string filepath = ros::package::getPath("extrinsic_calibration") + "/launch/frame_publisher.launch.yml";
	try{
		intrinsic_file = YAML::LoadFile(filepath);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mResults file \"%s\" does not exist or contains syntax errors.\e[31m\n", filepath.c_str());
		return 0;
	}
	try{
		response.rms = intrinsic_file["rms"].as<double>();
		response.l_drift_ang = intrinsic_file["b_delta_r"].as<double>();
		response.l_drift_pos = intrinsic_file["b_delta_m"].as<double>();
		response.r_drift_ang = intrinsic_file["t_delta_r"].as<double>();
		response.r_drift_pos = intrinsic_file["t_delta_m"].as<double>();
		response.c_drift_ang = intrinsic_file["c_delta_r"].as<double>();
		response.c_drift_pos = intrinsic_file["c_delta_m"].as<double>();
		
		response.forearm_to_cam.translation.x = intrinsic_file["c_x"].as<double>();
		response.forearm_to_cam.translation.y = intrinsic_file["c_y"].as<double>();
		response.forearm_to_cam.translation.z = intrinsic_file["c_z"].as<double>();
		
		Eigen::Quaterniond q;
		q =
			Eigen::AngleAxisd(intrinsic_file["c_w"].as<double>(), Eigen::Vector3d::UnitZ()) *
   			Eigen::AngleAxisd(intrinsic_file["c_p"].as<double>(), Eigen::Vector3d::UnitY()) *
   			Eigen::AngleAxisd(intrinsic_file["c_r"].as<double>(), Eigen::Vector3d::UnitX());
   		response.forearm_to_cam.rotation.x = q.x();
   		response.forearm_to_cam.rotation.y = q.y();
   		response.forearm_to_cam.rotation.z = q.z();
   		response.forearm_to_cam.rotation.w = q.w();
   		
		
	} catch(YAML::RepresentationException e){
		printf("\e[39mResult parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}

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
