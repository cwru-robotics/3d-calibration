#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "dewarp_common.cpp"

void dewarp_cb(const sensor_msgs::Image::ConstPtr & im){
	cv::Mat im_original;
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		im_original = cv_ptr->image;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
	
	cv::imshow("Original Image", im_original);
	
	cv::Mat im_remapped;
	undistort(im_original, im_remapped);
	
	cv::imshow("Dewarped Image", im_remapped);
	
	cv::waitKey(1);
}

int main(int argc, char ** argv){
	if(argc < 3){
		printf("\n\n usage: rosrun test_dewarp test_dewarp_topic /path/to/calibration.yml /image/topic\n\n");
		return 0;
	}
	if(!get_intrinsics(argv[1])){
		return 0;
	}
	
	ros::init(argc, argv, "dewarp_topic");
	ros::NodeHandle nh;
	
	cv::namedWindow("Original Image");
	cv::namedWindow("Dewarped Image");
	
	ros::Subscriber s = nh.subscribe(argv[2], 1, dewarp_cb);
	printf("Listening for images on %s.\n", argv[2]);
	
	ros::spin();

	return 0;
}
