#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <single_corner/single_corner.h>

namespace sc{
	cv::Point2d detect_single_corner(const cv::Mat& img){
	
		cv::Mat img_grayscale;
		cv::cvtColor(img, img_grayscale, cv::COLOR_BGR2GRAY);
		
		cv::Mat img_corner;
		cv::cornerHarris(img_grayscale, img_corner, 2, 3, 0.04);
		
		//Sometimes (especially in reconstructed PCL images) there are
		//corner-like artifacts at the very edges. So we just crop those
		//out. It's not like POIs should be anywhere near the edges of
		//image anyway.
		cv::Mat img_cropped = img_corner(cv::Rect(
			10, 10,
			img_corner.cols - 20, img_corner.rows - 20
		));
		
		cv::Mat img_corner_normed;
		cv::normalize(img_cropped, img_corner_normed, 0, 1.0, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
		
		cv::Mat corner_threshold;
		cv::threshold(img_corner_normed, corner_threshold, 0.9, 0.0, cv::THRESH_TOZERO);
		
		/*cv::imshow("GS", img_grayscale);
		cv::waitKey();
		cv::imshow("CH", img_corner_normed);
		cv::waitKey();
		cv::imshow("THRESH", corner_threshold);
		cv::waitKey();*/
		
		cv::Point2d wavg = {0.0, 0.0};
		double sum = 0.0;
		for(int x = 0; x < corner_threshold.cols; x++){
			for(int y = 0; y < corner_threshold.rows; y++){
				wavg += {x * corner_threshold.at<float>(y, x), y * corner_threshold.at<float>(y, x)};
				sum += corner_threshold.at<float>(y, x);
			}
		}

		
		wavg /= sum;
		
		//Undo the displacement caused by the crop.
		wavg += {10.0, 10.0};
		
		return wavg;
	}
}
