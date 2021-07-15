#include "dewarp_common.cpp"


int main(int argc, char ** argv){
	if(argc < 3){
		printf("\n\n usage: rosrun test_dewarp test_dewarp_file /path/to/calibration.yml /path/to/image/folder\n\n");
		return 0;
	}
	if(!get_intrinsics(argv[1])){
		return 0;
	}
	
	cv::Mat img = cv::imread(argv[2], cv::IMREAD_COLOR);
	if(! img.data ){// Check for invalid input
		printf("\e[31mCould not process image %s.\e[39m\n", argv[2]);
		return 0;
	}
	
	cv::namedWindow("Original Image");
	cv::namedWindow("Dewarped Image");
	
	cv::imshow("Original Image", img);
	
	cv::Mat im_remapped;
	undistort(img, im_remapped);
	
	cv::imshow("Dewarped Image", im_remapped);
	
	cv::waitKey();
	
	return 0;
}
