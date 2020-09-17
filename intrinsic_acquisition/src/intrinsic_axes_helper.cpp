#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

std::string x_map = "x+";
std::string y_map = "y+";
std::string z_map = "z+";

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

	//Show the image with the things drawn on it.
	cv::imshow("Camera and Axes", img);
	cv::waitKey(1);
}

int main(int argc, char ** argv){

	//ROS initialization
	ros::init(argc, argv, "axis_helper");
	ros::NodeHandle nh;

	if(argc < 2){
		ROS_ERROR("USAGE: rosrun intrinsic_axis_helper /camera/topic [/path/to/ADF]");
		return 0;
	}
	
	cv::namedWindow("Camera and Axes");
	
	ros::Subscriber imsub = nh.subscribe(argv[1], 1, new_frame_CB);
	
	ROS_INFO("Axis helper has subscribed to %s and is waiting for images.", argv[1]);
	
	ros::spin();
	
	cv::destroyAllWindows();
	return 0;
}
