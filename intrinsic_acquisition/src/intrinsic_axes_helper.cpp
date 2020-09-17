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
	
	//Calculate some generally useful elements
	int u_max = img.cols;
	int v_max = img.rows;
	
	int u_center = u_max / 2;
	int v_center = v_max / 2;
	
	int tenth = std::min(u_max, v_max) / 10;
	
	//Draw the camera frame
	cv::arrowedLine(
		img,
		cv::Point(u_center, v_center),
		cv::Point(u_center + tenth, v_center),
		cv::Scalar(0, 0, 200),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera X",
		cv::Point(u_center + tenth, v_center),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 0, 200)
	);
	
	cv::arrowedLine(
		img,
		cv::Point(u_center, v_center),
		cv::Point(u_center, v_center + tenth),
		cv::Scalar(0, 200, 0),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera Y",
		cv::Point(u_center + 10, v_center + tenth),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(0, 200, 0)
	);
	
	cv::arrowedLine(
		img,
		cv::Point(u_center, v_center),
		cv::Point(u_center - tenth / 4, v_center - tenth / 4),
		cv::Scalar(200, 0, 0),//BGR for some reason
		2//Thickness
	);
	cv::putText(
		img,
		"Camera Z",
		cv::Point(u_center - tenth / 4, v_center - tenth / 4),
		cv::FONT_HERSHEY_COMPLEX,
		0.5,//Font size
		cv::Scalar(200, 0, 0)
	);

	//Show the image with the things drawn on it.
	cv::imshow("Camera and Axes", img);
	//For some reason. image won't display unless there is a nominal wait
	//time between imshow() and the end of the function.
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
