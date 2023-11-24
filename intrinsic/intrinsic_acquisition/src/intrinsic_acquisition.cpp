#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

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
	while(!ros::service::call("/motion_command", srv) && ros::ok()){
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
std::vector<cv::Mat> imgs;
int image_index;
void imcb(const sensor_msgs::Image::ConstPtr & im){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		imgs[image_index] = cv_ptr->image;
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
	
	max_x -= fmod(max_x - min_x, inc_x);
	max_y -= fmod(max_y - min_y, inc_y);
	max_z -= fmod(max_z - min_z, inc_z);
	
	ROS_INFO("Successfully initialized test from TDF \"%s\".", argv[2]);
	
	std::vector<std::string> names;
	boost::split(names, argv[1], boost::is_any_of(","));
	
	printf("Will listen for %lu cameras.\n", names.size());
	
	if(names.size() < 1){
		return 0;
	}
	
	std::string folder = std::string(argv[2]);
	std::string filename = folder.substr(folder.find_last_of("/\\"));
	folder = folder.substr(0, folder.find_last_of("/\\"));
	boost::filesystem::path master_path = boost::filesystem::path(folder);//TODO Do this with Boost instead of string manipulation.
	ROS_INFO("Data will be saved at root directory %s.", folder.c_str());
	for(std::string name : names){
		printf("\t%s\t:\t", name.c_str());
		ros::topic::waitForMessage<sensor_msgs::Image>(name, nh);
		//Can't write filenames with the slashes ROS uses to describe topics, so replace them with spaces (a character that cannot exist in a ROS topic):
		std::string topic_file = replaceChar(name, '/', ' ');
		try{
			boost::filesystem::create_directory(master_path / boost::filesystem::path(topic_file));
			boost::filesystem::copy(argv[2], master_path / boost::filesystem::path(topic_file) / boost::filesystem::path(filename));
		} catch(std::exception e){
		}
		printf("OK\n");
	}
	
	//TODO There is a more efficient to maneuver around but harder to
	//algorithmically generate search pattern involving a series of
	//expanding cubical shells.
	int i = 0;
	double total = 
		(((max_z - min_z) / inc_z) + 1) * 
		(((max_y - min_y) / inc_y) + 1) *
		(((max_x - min_x) / inc_x) + 1)
	;
	for(double x = min_x; x <= max_x; x += inc_x){
		for(double y = min_y; y <= max_y; y += inc_y){
			for(double z = min_z; z <= max_z; z += inc_z){
				
				printf("\n%f%%: Move sled to x=%f y=%f z=%f and then press s to skip or other to acquire: ", (i / total) * 100.0, x, y, z);
				
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
					for(std::string name : names){
						sensor_msgs::Image::ConstPtr image_topic = ros::topic::waitForMessage<sensor_msgs::Image>(name, nh);
						cv_bridge::CvImagePtr cv_ptr;
						cv::Mat img;
						try {
							cv_ptr = cv_bridge::toCvCopy(image_topic);
							img = cv_ptr->image;
						} catch (cv_bridge::Exception &e) {
							ROS_ERROR("Could not convert from encoding to 'bgr8'.");
							continue;
						}
						
						std::string x_code = replaceChar(std::to_string(x), '.', 'p');
						std::string y_code = replaceChar(std::to_string(y), '.', 'p');
						std::string z_code = replaceChar(std::to_string(z), '.', 'p');
					
						cv::namedWindow(name);
						cv::imshow(name, img);
						cv::waitKey(500);
						cv::destroyAllWindows();
						
						cv::imwrite(
							folder + "/" + replaceChar(name, '/', ' ') + "/img_"+x_code+"_"+y_code+"_"+z_code+".png",
							img
						);
						//printf("%s\n", (folder + "/" +  replaceChar(name, '/', ' ') + "/img_"+x_code+"_"+y_code+"_"+z_code+".png").c_str());
					}
				}
				if(!ros::ok()){
					return 0;
				}
				i++;
			}
		}
	}
	
	return 0;
}
