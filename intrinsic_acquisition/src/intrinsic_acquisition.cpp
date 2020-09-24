#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <turtlesim/Spawn.h>

#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

//TODO This is used in multiple locations. Should it be moved into its own library?
//TODO It is also not especially efficient.
std::string replaceChar(std::string str, char ch1, char ch2) {
	for (int i = 0; i < str.length(); ++i) {
		if (str[i] == ch1) str[i] = ch2;
	}
	return str;
}

bool ready_to_record;
bool skip;
void topic_thread(double x, double y, double z){
	turtlesim::Spawn srv;
	srv.request.x = x;
	srv.request.y = y;
	srv.request.theta = z;
	while(!ros::service::call("/motion_command", srv) && ! ros::ok()){
		ros::spinOnce();
	}
	ready_to_record = true;
	skip = !srv.response.name.empty();
}
void key_thread(){
	char code = std::getchar();
	ready_to_record = true;
	skip = code == 's';
}

bool got_image;
cv::Mat img;
void imcb(const sensor_msgs::Image::ConstPtr & im){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		img = cv_ptr->image;
		got_image = true;
		return;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "axis_helper");
	ros::NodeHandle nh;

	if(argc < 3){
		ROS_ERROR("USAGE: rosrun intrinsic_acquisition /camera/topic /path/to/ADF");
		return 0;
	}
	
	YAML::Node data_file;
	try{
		data_file = YAML::LoadFile(argv[2]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		ROS_ERROR("TDF \"%s\" does not exist or contains syntax errors.", argv[2]);
		return 0;
	}
	
	if(!data_file["target_x"]){
		ROS_ERROR("TDF \"%s\" is missing its target_x field.", argv[2]);
		return 0;
	}
	if(!data_file["target_y"]){
		ROS_ERROR("TDF \"%s\" is missing its target_y field.", argv[2]);
		return 0;
	}
	if(!data_file["target_spacing"]){
		ROS_ERROR("TDF \"%s\" is missing its target_spacing field.", argv[2]);
		return 0;
	}
	
	if(!data_file["sled_x"]){
		ROS_ERROR("TDF \"%s\" is missing its sled_x field.", argv[2]);
		return 0;
	}
	if(!data_file["sled_y"]){
		ROS_ERROR("TDF \"%s\" is missing its sled_y field.", argv[2]);
		return 0;
	}
	if(!data_file["sled_z"]){
		ROS_ERROR("TDF \"%s\" is missing its sled_z field.", argv[2]);
		return 0;
	}
	
	
	if(!data_file["min_sled_x"]){
		ROS_ERROR("TDF \"%s\" is missing its min_sled_x field.", argv[2]);
		return 0;
	}
	double min_x = data_file["min_sled_x"].as<double>();
	if(!data_file["max_sled_x"]){
		ROS_ERROR("TDF \"%s\" is missing its max_sled_x field.", argv[2]);
		return 0;
	}
	double max_x = data_file["max_sled_x"].as<double>();
	if(!data_file["inc_sled_x"]){
		ROS_ERROR("TDF \"%s\" is missing its inc_sled_x field.", argv[2]);
		return 0;
	}
	double inc_x = data_file["inc_sled_x"].as<double>();
	
	if(!data_file["min_sled_y"]){
		ROS_ERROR("TDF \"%s\" is missing its min_sled_y field.", argv[2]);
		return 0;
	}
	double min_y = data_file["min_sled_y"].as<double>();
	if(!data_file["max_sled_y"]){
		ROS_ERROR("TDF \"%s\" is missing its max_sled_y field.", argv[2]);
		return 0;
	}
	double max_y = data_file["max_sled_y"].as<double>();
	if(!data_file["inc_sled_y"]){
		ROS_ERROR("TDF \"%s\" is missing its inc_sled_y field.", argv[2]);
		return 0;
	}
	double inc_y = data_file["inc_sled_y"].as<double>();
	
	if(!data_file["min_sled_z"]){
		ROS_ERROR("TDF \"%s\" is missing its min_sled_z field.", argv[2]);
		return 0;
	}
	double min_z = data_file["min_sled_z"].as<double>();
	if(!data_file["max_sled_z"]){
		ROS_ERROR("TDF \"%s\" is missing its max_sled_z field.", argv[2]);
		return 0;
	}
	double max_z = data_file["max_sled_z"].as<double>();
	if(!data_file["inc_sled_z"]){
		ROS_ERROR("TDF \"%s\" is missing its inc_sled_z field.", argv[2]);
		return 0;
	}
	double inc_z = data_file["inc_sled_z"].as<double>();
	
	if(min_x > max_x){
		ROS_ERROR("min_x > max_x");
		return 0;
	}
	if(min_y > max_y){
		ROS_ERROR("min_y > max_y");
		return 0;
	}
	if(min_z > max_z){
		ROS_ERROR("min_z > max_z");
		return 0;
	}
	
	if(inc_x <= 0.0){
		ROS_ERROR("X increment should be positive.");
		return 0;
	}
	if(inc_y <= 0.0){
		ROS_ERROR("Y increment should be positive.");
		return 0;
	}
	if(inc_z <= 0.0){
		ROS_ERROR("Z increment should be positive.");
		return 0;
	}
	
	ROS_INFO("Successfully initialized test from TDF \"%s\".", argv[2]);
	
	ros::Subscriber img_sub = nh.subscribe(argv[1], 1, imcb);
	
	got_image = false;
	while(!got_image && ros::ok()){
		ros::spinOnce();
		ROS_WARN("Waiting for image data on topic %s.", argv[1]);
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("Recieving images on topic %s.", argv[1]);
	ROS_INFO("Resolution is %d by %d.", img.rows, img.cols);
	data_file["res_x"] = img.rows;
	data_file["res_y"] = img.cols;
	std::ofstream e_out;
	e_out.open(argv[2]);
	e_out << data_file;
	e_out.close();
	
	std::string data_path = std::string(argv[2]);
	data_path = data_path.substr(0, data_path.find_last_of("\\/"));
	ROS_INFO("Data will be saved at %s.", data_path.c_str());
	
	
	//TODO There is a more efficient to maneuver around but harder to
	//algorithmically generate search pattern involving a series of
	//expanding cubical shells.
	int i = 0;
	for(double x = min_x; x <= max_x; x += inc_x){
		for(double y = min_y; y <= max_y; y += inc_y){
			for(double z = min_z; z <= max_z; z += inc_z){
				
				printf("\n%f%%: Move sled to x=%f y=%f z=%f and then press s to skip or other to acquire: ", (i / ((double) x * y * z)), x, y, z);
				
				std::thread click_thread(key_thread);
				std::thread service_thread(topic_thread, x, y, z);
				ready_to_record = false;
				skip = true;
				while(!ready_to_record && ros::ok()){
					ros::spinOnce();
					//motion_publisher.publish(p);
				}
				click_thread.detach();
				service_thread.detach();
				
				if(!skip){
					got_image = false;
					while(!got_image && ros::ok()){
						ros::spinOnce();
					}
					
					std::string x_code = replaceChar(std::to_string(x), '.', 'p');
					std::string y_code = replaceChar(std::to_string(y), '.', 'p');
					std::string z_code = replaceChar(std::to_string(z), '.', 'p');
					
					cv::namedWindow("Captured image");
					cv::imshow("Captured image", img);
					cv::waitKey(1000);
					cv::destroyAllWindows();
					
					cv::imwrite(
						data_path + "/img_"+x_code+"_"+y_code+"_"+z_code+".png",
						img
					);
				}
			}
		}
	}
	
	return 0;
}
