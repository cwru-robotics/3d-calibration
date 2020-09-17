#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "yaml-cpp/yaml.h"

#define BUTTON_SIZE 50

std::string x_map = "x+";
std::string y_map = "y+";
std::string z_map = "z+";

cv::Mat control_panel;

std::string file_path;
YAML::Node data_file;

void new_frame_CB(const sensor_msgs::Image::ConstPtr & im){
	//Get the new image.
	cv::Mat img;
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		img = cv_ptr->image;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
	
	//Calculate some generally useful elements
	int u_max = img.cols;
	int v_max = img.rows;
	
	int u_center = u_max / 2;
	int v_center = v_max / 2;
	
	int tenth = std::min(u_max, v_max) / 10;
	
	//Draw the camera frame
	cv::Point p_x = cv::Point( tenth, 0);
	cv::Point n_x = cv::Point(-tenth, 0);
	cv::Point p_y = cv::Point(0,  tenth);
	cv::Point n_y = cv::Point(0, -tenth);
	cv::Point p_z = cv::Point(-tenth / 4, -tenth / 4);
	cv::Point n_z = cv::Point( tenth / sqrt(2),  tenth / sqrt(2));
	
	cv::Point c_origin = cv::Point(u_center, v_center);
	
	cv::arrowedLine(
		img,
		c_origin,
		c_origin + p_x,
		cv::Scalar(0, 0, 200),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera X",
		c_origin + p_x,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 200)
	);
	
	cv::arrowedLine(
		img,
		c_origin,
		c_origin + p_y,
		cv::Scalar(0, 200, 0),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera Y",
		c_origin + p_y,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 200, 0)
	);
	
	cv::arrowedLine(
		img,
		c_origin,
		c_origin + p_z,
		cv::Scalar(200, 0, 0),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera Z",
		c_origin + p_z,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(200, 0, 0)
	);
	
	//Draw the motion frame
	cv::Point cam_x_terminus;
	cv::Point cam_y_terminus;
	cv::Point cam_z_terminus;
	
	if(x_map[1] == '+'){
		switch(x_map[0]){
			case 'x': cam_x_terminus = p_x; break;
			case 'y': cam_x_terminus = p_y; break;
			case 'z': cam_x_terminus = p_z; break;
		}
	} else {
		switch(x_map[0]){
			case 'x': cam_x_terminus = n_x; break;
			case 'y': cam_x_terminus = n_y; break;
			case 'z': cam_x_terminus = n_z; break;
		}
	}
	if(y_map[1] == '+'){
		switch(y_map[0]){
			case 'x': cam_y_terminus = p_x; break;
			case 'y': cam_y_terminus = p_y; break;
			case 'z': cam_y_terminus = p_z; break;
		}
	} else {
		switch(y_map[0]){
			case 'x': cam_y_terminus = n_x; break;
			case 'y': cam_y_terminus = n_y; break;
			case 'z': cam_y_terminus = n_z; break;
		}
	}
	if(z_map[1] == '+'){
		switch(z_map[0]){
			case 'x': cam_z_terminus = p_x; break;
			case 'y': cam_z_terminus = p_y; break;
			case 'z': cam_z_terminus = p_z; break;
		}
	} else {
		switch(z_map[0]){
			case 'x': cam_z_terminus = n_x; break;
			case 'y': cam_z_terminus = n_y; break;
			case 'z': cam_z_terminus = n_z; break;
		}
	}
	
	cv::Point s_origin = cv::Point(u_center - tenth * 2, v_center - tenth * 2);
	
	cv::arrowedLine(
		img,
		s_origin,
		s_origin + cam_x_terminus,
		cv::Scalar(200, 0, 200),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Sled X",
		s_origin + cam_x_terminus,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(200, 0, 200)
	);
	
	cv::arrowedLine(
		img,
		s_origin,
		s_origin + cam_y_terminus,
		cv::Scalar(0, 200, 200),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Sled Y",
		s_origin + cam_y_terminus,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 200, 200)
	);
	
	cv::arrowedLine(
		img,
		s_origin,
		s_origin + cam_z_terminus,
		cv::Scalar(200, 200, 0),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Sled Z",
		s_origin + cam_z_terminus,
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(200, 200, 0)
	);

	//Show the image with the things drawn on it.
	cv::imshow("Camera and Axes", img);
	cv::imshow("Controls", control_panel);
	//For some reason. image won't display unless there is a nominal wait
	//time between imshow() and the end of the function.
	cv::waitKey(1);
}

void cp_draw(){
	//Save bar.
	cv::putText(
		control_panel,
		"SAVE",
		cv::Point(BUTTON_SIZE, BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(255, 255, 255)
	);
	
	cv::Scalar x_color;
	if(x_map[0] == 'x'){
		x_color = cv::Scalar(0, 0, 255);
	} else if(x_map[0] == 'y'){
		x_color = cv::Scalar(0, 255, 0);
	} else{
		x_color = cv::Scalar(255, 0, 0);
	}
	
	cv::Scalar y_color;
	if(y_map[0] == 'x'){
		y_color = cv::Scalar(0, 0, 255);
	} else if(y_map[0] == 'y'){
		y_color = cv::Scalar(0, 255, 0);
	} else{
		y_color = cv::Scalar(255, 0, 0);
	}
	
	cv::Scalar z_color;
	if(z_map[0] == 'x'){
		z_color = cv::Scalar(0, 0, 255);
	} else if(z_map[0] == 'y'){
		z_color = cv::Scalar(0, 255, 0);
	} else{
		z_color = cv::Scalar(255, 0, 0);
	}
	
	//x bar
	cv::rectangle(
		control_panel,
		cv::Point(0, BUTTON_SIZE),
		cv::Point(BUTTON_SIZE, BUTTON_SIZE * 2),
		cv::Scalar(200, 0, 200),
		cv::FILLED
	);
	cv::putText(
		control_panel,
		"X",
		cv::Point(BUTTON_SIZE/4, 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(BUTTON_SIZE, BUTTON_SIZE),
		cv::Point(BUTTON_SIZE * 2, BUTTON_SIZE * 2),
		x_color / 2,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({x_map[1]}),
		cv::Point(BUTTON_SIZE + BUTTON_SIZE/4, 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(255, 255, 255)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(2 * BUTTON_SIZE, BUTTON_SIZE),
		cv::Point(BUTTON_SIZE * 3, BUTTON_SIZE * 2),
		x_color,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({x_map[0]}),
		cv::Point(2 * BUTTON_SIZE + BUTTON_SIZE/4, 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
	
	//y bar
	cv::rectangle(
		control_panel,
		cv::Point(0, BUTTON_SIZE * 2),
		cv::Point(BUTTON_SIZE, BUTTON_SIZE * 3),
		cv::Scalar(0, 200, 200),
		cv::FILLED
	);
	cv::putText(
		control_panel,
		"Y",
		cv::Point(BUTTON_SIZE/4, BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(BUTTON_SIZE, BUTTON_SIZE * 2),
		cv::Point(BUTTON_SIZE * 2, BUTTON_SIZE * 3),
		y_color / 2,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({y_map[1]}),
		cv::Point(BUTTON_SIZE + BUTTON_SIZE/4, BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(255, 255, 255)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(2 * BUTTON_SIZE, BUTTON_SIZE * 2),
		cv::Point(BUTTON_SIZE * 3, BUTTON_SIZE * 3),
		y_color,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({y_map[0]}),
		cv::Point(2 * BUTTON_SIZE + BUTTON_SIZE/4, BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
	
	//z bar
	cv::rectangle(
		control_panel,
		cv::Point(0, BUTTON_SIZE * 3),
		cv::Point(BUTTON_SIZE, BUTTON_SIZE * 4),
		cv::Scalar(200, 200, 0),
		cv::FILLED
	);
	cv::putText(
		control_panel,
		"Z",
		cv::Point(BUTTON_SIZE/4, 2 * BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(BUTTON_SIZE, BUTTON_SIZE * 3),
		cv::Point(BUTTON_SIZE * 2, BUTTON_SIZE * 4),
		z_color / 2,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({z_map[1]}),
		cv::Point(BUTTON_SIZE + BUTTON_SIZE/4, 2 * BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(255, 255, 255)
	);
	
	cv::rectangle(
		control_panel,
		cv::Point(2 * BUTTON_SIZE, BUTTON_SIZE * 3),
		cv::Point(BUTTON_SIZE * 3, BUTTON_SIZE * 4),
		z_color,
		cv::FILLED
	);
	cv::putText(
		control_panel,
		std::string({z_map[0]}),
		cv::Point(2 * BUTTON_SIZE + BUTTON_SIZE/4, 2 * BUTTON_SIZE + 3 * BUTTON_SIZE / 2),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 0)
	);
}

void save_behavior(){
	if(file_path.empty()){
		ROS_WARN("No TDF argument given. Saving is disabled.");
		return;
	}
	
	data_file["sled_x"] = x_map;
	data_file["sled_y"] = y_map;
	data_file["sled_z"] = z_map;
	
	std::ofstream e_out;
	e_out.open(file_path);
	e_out << data_file;
	e_out.close();
	
	printf("Saved map \n");
	printf("\tsled_x: %s\n", x_map.c_str());
	printf("\tsled_y: %s\n", y_map.c_str());
	printf("\tsled_z: %s\n", z_map.c_str());
	printf("to file %s.\n", file_path.c_str());
}

void pm_behavior(std::string * element){
	if((*element)[1] == '+'){
		(*element)[1] = '-';
	} else {
		(*element)[1] = '+';
	}
	cp_draw();
}

void xyz_behavior(std::string * element){
	if((*element)[0] == 'x'){
		(*element)[0] = 'z';
	} else if((*element)[0] == 'y'){
		(*element)[0] = 'x';
	} else{
		(*element)[0] = 'y';
	}
	cp_draw();
}

void control_click_cb(int event, int x, int y, int flags, void* param){
	if(event != cv::EVENT_LBUTTONDOWN){//We only want left down clicks.
		return;
	}
	
	if(y < BUTTON_SIZE){
		save_behavior();
		return;
	}
	
	std::string * operand;
	if(y > BUTTON_SIZE && y < BUTTON_SIZE * 2){
		operand = &x_map;
	} else if(y < BUTTON_SIZE * 3){
		operand = &y_map;
	} else{
		operand = &z_map;
	}
	
	if(x > BUTTON_SIZE && x < BUTTON_SIZE * 2){
		pm_behavior(operand);
	} else if(x > BUTTON_SIZE * 2){
		xyz_behavior(operand);
	}
}

int main(int argc, char ** argv){

	//ROS initialization
	ros::init(argc, argv, "axis_helper");
	ros::NodeHandle nh;

	if(argc < 2){
		ROS_ERROR("USAGE: rosrun intrinsic_axis_helper /camera/topic [/path/to/ADF]");
		return 0;
	}
	
	if(argc < 3){
		ROS_WARN("No TDF argument given. Saving will be disabled and default values will be used.");
		file_path = "";
	} else{
		file_path = std::string(argv[2]);
		try{
			data_file = YAML::LoadFile(file_path);
			
			if(!data_file["sled_x"]){
				ROS_WARN("TDF \"%s\" does not contain a sled_x value.", file_path.c_str());
				throw YAML::BadFile();
			}
			if(!data_file["sled_y"]){
				ROS_WARN("TDF \"%s\" does not contain a sled_y value.", file_path.c_str());
				throw YAML::BadFile();
			}
			if(!data_file["sled_z"]){
				ROS_WARN("TDF \"%s\" does not contain a sled_z value.", file_path.c_str());
				throw YAML::BadFile();
			}
			
			std::string candidate;
			candidate = data_file["sled_x"].as<std::string>();
			if(
				candidate.length() != 2 ||
				(candidate[0] != 'x' && candidate[0] != 'y' && candidate[0] != 'z') ||
				(candidate[1] != '+' && candidate[1] != '-')
			){
				ROS_WARN("Imparisble sled_x value \"%s\".", candidate.c_str());
				throw YAML::BadFile();
			} else{
				x_map = candidate;
			}
			candidate = data_file["sled_y"].as<std::string>();
			if(
				candidate.length() != 2 ||
				(candidate[0] != 'x' && candidate[0] != 'y' && candidate[0] != 'z') ||
				(candidate[1] != '+' && candidate[1] != '-')
			){
				ROS_WARN("Imparisble sled_y value \"%s\".", candidate.c_str());
				throw YAML::BadFile();
			} else{
				y_map = candidate;
			}
			candidate = data_file["sled_z"].as<std::string>();
			if(
				candidate.length() != 2 ||
				(candidate[0] != 'x' && candidate[0] != 'y' && candidate[0] != 'z') ||
				(candidate[1] != '+' && candidate[1] != '-')
			){
				ROS_WARN("Imparisble sled_z value \"%s\".", candidate.c_str());
				throw YAML::BadFile();
			} else{
				z_map = candidate;
			}
			ROS_INFO("Successfully initialized mappings from TDF.");
				
		} catch(YAML::BadFile e){
			ROS_WARN("TDF \"%s\" does not exist or does not contain valid axis mapping values. Default values will be created.", file_path.c_str());
			data_file["sled_x"] = x_map;
			data_file["sled_y"] = y_map;
			data_file["sled_z"] = z_map;
		}
		
	}
	
	cv::namedWindow("Camera and Axes");
	
	cv::namedWindow("Controls");
	control_panel = cv::Mat(BUTTON_SIZE * 4, BUTTON_SIZE * 3, CV_8UC3, cv::Scalar(0, 0, 0));
	cp_draw();
	cv::setMouseCallback("Controls", control_click_cb);
	
	ros::Subscriber imsub = nh.subscribe(argv[1], 1, new_frame_CB);
	
	ROS_INFO("Axis helper has subscribed to %s and is waiting for images.", argv[1]);
	
	ros::spin();
	
	cv::destroyAllWindows();
	return 0;
}
