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
