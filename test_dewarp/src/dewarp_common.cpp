#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "yaml-cpp/yaml.h"

cv::Mat map1, map2;

bool get_intrinsics(const char * location){
	YAML::Node data_file;
	try{
		data_file = YAML::LoadFile(location);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[91mCal file \"%s\" does not exist or contains syntax errors.\e[39m\n", location);
		return false;
	}
	double k1, k2, k3, p1, p2;
	double fx, fy, cx, cy;
	double rx, ry;
	try{
		k1 = data_file["k1"].as<double>();
		k2 = data_file["k2"].as<double>();
		k3 = data_file["k3"].as<double>();
		p1 = data_file["p1"].as<double>();
		p2 = data_file["p2"].as<double>();
		
		fx = data_file["fx"].as<double>();
		fy = data_file["fy"].as<double>();
		cx = data_file["cx"].as<double>();
		cy = data_file["cy"].as<double>();
		
		rx = data_file["rx"].as<double>();
		ry = data_file["ry"].as<double>();
	} catch(YAML::Exception e){
		printf("\e[91m%s\e[39m\n", e.what());
		return false;
	}
	
	cv::Mat K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat D = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
	cv::initUndistortRectifyMap(K, D, cv::Mat(), K,  cv::Size(rx, ry), CV_32FC1,  map1,  map2);
	
	return true;
}

void undistort(const cv::Mat & i_in, cv::Mat & i_out){
	cv::remap(i_in, i_out, map1, map2, cv::INTER_LINEAR);
}
