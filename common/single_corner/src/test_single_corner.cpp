#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <single_corner/single_corner.h>

using namespace std;

int main(int argc, char ** argv){

	cv::Mat img = cv::imread(argv[1]);

	cv::Point2d tp1 = sc::detect_single_corner(img);
	printf("\nFound corner location (%f, %f).\n\n", tp1.x, tp1.y);

	// cv::drawMarker(
	// 	img,
	// 	tp,
	// 	cv::Scalar(255, 0, 0),//Marker color
	// 	cv::MARKER_TILTED_CROSS, 5//Type and size.
	// );
	
	// cv::imshow("Result", img);

	// if (cv::waitKey())	cv::destroyAllWindows();

	return 0;
}
