#ifndef SINGLE_CORNER
#define SINGLE_CORNER

#include <opencv2/core.hpp>

namespace sc{
	cv::Point2d detect_single_corner(const cv::Mat& img);	
}
#endif
