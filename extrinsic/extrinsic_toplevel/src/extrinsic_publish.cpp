#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

geometry_msgs::TransformStamped the_current_calibration;

std::string confirmed_file_path;

bool CB_cal_update(
	std_srvs::Empty::Request& request,
	std_srvs::Empty::Response& response
){
	std::string tentative_file_path = ros::package::getPath("extrinsic_calibration") + "/launch/frame_publisher.launch.yml";
	YAML::Node tentative_file;
	try{
		tentative_file = YAML::LoadFile(tentative_file_path);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mResults file \"%s\" does not exist or contains syntax errors.\e[31m\n", tentative_file_path.c_str());
		return 0;
	}
	double new_c_x, new_c_y, new_c_z, new_c_r, new_c_p, new_c_w;
	double new_b_x, new_b_y, new_b_z, new_b_r, new_b_p, new_b_w;
	double new_t_x, new_t_y, new_t_z, new_t_r, new_t_p, new_t_w;
	try{
		new_c_x = tentative_file["c_x"].as<double>();
		new_c_y = tentative_file["c_y"].as<double>();
		new_c_z = tentative_file["c_z"].as<double>();
		new_c_r = tentative_file["c_r"].as<double>();
		new_c_p = tentative_file["c_p"].as<double>();
		new_c_w = tentative_file["c_w"].as<double>();
		
		new_b_x = tentative_file["b_x"].as<double>();
		new_b_y = tentative_file["b_y"].as<double>();
		new_b_z = tentative_file["b_z"].as<double>();
		new_b_r = tentative_file["b_r"].as<double>();
		new_b_p = tentative_file["b_p"].as<double>();
		new_b_w = tentative_file["b_w"].as<double>();
		
		new_t_x = tentative_file["t_x"].as<double>();
		new_t_y = tentative_file["t_y"].as<double>();
		new_t_z = tentative_file["t_z"].as<double>();
		new_t_r = tentative_file["t_r"].as<double>();
		new_t_p = tentative_file["t_p"].as<double>();
		new_t_w = tentative_file["t_w"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mResult parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	
	std::ofstream confirmed_file_output;
	confirmed_file_output.open(confirmed_file_path);
	if(!confirmed_file_output){
		printf("\e[39mResults file \"%s\" does not exist or contains syntax errors.\e[31m\n", confirmed_file_path.c_str());
		return false;
	}
	
	confirmed_file_output << "cam_x: " << new_c_x << "\n";
	confirmed_file_output << "cam_y: " << new_c_y << "\n";
	confirmed_file_output << "cam_z: " << new_c_z << "\n";
	confirmed_file_output << "cam_r: " << new_c_r << "\n";
	confirmed_file_output << "cam_p: " << new_c_p << "\n";
	confirmed_file_output << "cam_w: " << new_c_w << "\n";
	
	confirmed_file_output << "top_x: " << new_t_x << "\n";
	confirmed_file_output << "top_y: " << new_t_y << "\n";
	confirmed_file_output << "top_z: " << new_t_z << "\n";
	confirmed_file_output << "top_r: " << new_t_r << "\n";
	confirmed_file_output << "top_p: " << new_t_p << "\n";
	confirmed_file_output << "top_w: " << new_t_w << "\n";
	
	confirmed_file_output << "bot_x: " << new_b_x << "\n";
	confirmed_file_output << "bot_y: " << new_b_y << "\n";
	confirmed_file_output << "bot_z: " << new_b_z << "\n";
	confirmed_file_output << "bot_r: " << new_b_r << "\n";
	confirmed_file_output << "bot_p: " << new_b_p << "\n";
	confirmed_file_output << "bot_w: " << new_b_w << "\n";
	
	confirmed_file_output.close();
	
	
	//Update the transform we will be publishing
	the_current_calibration.transform.translation.x = new_c_x;
	the_current_calibration.transform.translation.y = new_c_y;
	the_current_calibration.transform.translation.z = new_c_z;
		
	Eigen::Quaterniond q;
	q =
		Eigen::AngleAxisd(new_c_w, Eigen::Vector3d::UnitZ()) *
   		Eigen::AngleAxisd(new_c_p, Eigen::Vector3d::UnitY()) *
   		Eigen::AngleAxisd(new_c_r, Eigen::Vector3d::UnitX());
   	the_current_calibration.transform.rotation.x = q.x();
   	the_current_calibration.transform.rotation.y = q.y();
   	the_current_calibration.transform.rotation.z = q.z();
   	the_current_calibration.transform.rotation.w = q.w();
   	
   	the_current_calibration.header.stamp = ros::Time::now();
   		
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	static_broadcaster.sendTransform(the_current_calibration);

	return true;
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "cam_extrinsic_pub");
	ros::NodeHandle nh;
	
	
	//Load in known intrinsics for publication.
	confirmed_file_path = ros::package::getPath("cam_cal") + "/calibrations/stella_external.yml";
	
	YAML::Node confirmed_file;
	try{
		confirmed_file = YAML::LoadFile(confirmed_file_path);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mResults file \"%s\" does not exist or contains syntax errors.\e[31m\n", confirmed_file_path.c_str());
		return 0;
	}
	try{
		the_current_calibration.transform.translation.x = confirmed_file["cam_x"].as<double>();
		the_current_calibration.transform.translation.y = confirmed_file["cam_y"].as<double>();
		the_current_calibration.transform.translation.z = confirmed_file["cam_z"].as<double>();
		
		Eigen::Quaterniond q;
		q =
			Eigen::AngleAxisd(confirmed_file["cam_w"].as<double>(), Eigen::Vector3d::UnitZ()) *
   			Eigen::AngleAxisd(confirmed_file["cam_p"].as<double>(), Eigen::Vector3d::UnitY()) *
   			Eigen::AngleAxisd(confirmed_file["cam_r"].as<double>(), Eigen::Vector3d::UnitX());
   		the_current_calibration.transform.rotation.x = q.x();
   		the_current_calibration.transform.rotation.y = q.y();
   		the_current_calibration.transform.rotation.z = q.z();
   		the_current_calibration.transform.rotation.w = q.w();
	} catch(YAML::RepresentationException e){
		printf("\e[39mResult parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	
	the_current_calibration.header.stamp = ros::Time::now();
	the_current_calibration.header.frame_id = "forearm";
	the_current_calibration.child_frame_id = "cam_calibrated";
	
	
	//Fire up the broadcaster
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	static_broadcaster.sendTransform(the_current_calibration);
	
	ros::ServiceServer cal = nh.advertiseService("/update_extrinsic", CB_cal_update);
	
	ros::spin();

	return 0;
}
