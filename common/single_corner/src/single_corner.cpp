#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <single_corner/single_corner.h>

namespace sc{
	cv::Point2d detect_single_corner(const cv::Mat& img){
	
cv::Mat dst, cdst, cdstP;
	//Blur the image to eliminate thin lines; the Hough really likes these
	//but they are not the same thing as edges of checkerboards.
	cv::Mat soft;
	cv::blur(img, soft, cv::Size(5,5));

	//cv::imshow("Blurred", soft);

	//Perform edge detection
	cv::Mat ow;
	Canny(soft, ow, 50, 200, 3);

	//cv::imshow("Edges", ow);

	//Hough transform
	std::vector<cv::Vec2f> lines;
	HoughLines(ow, lines, 1, CV_PI/180, 150, 0, 0 );

	// Draw all the lines
	cv::Mat all = img.clone();
	for(int i = 0; i < lines.size(); i++ ){
        	float rho = lines[i][0];
		float theta = lines[i][1];
        	cv::Point pt1, pt2;
        	double a = cos(theta), b = sin(theta);
        	double x0 = a*rho, y0 = b*rho;
        	pt1.x = cvRound(x0 + 1000*(-b));
        	pt1.y = cvRound(y0 + 1000*(a));
        	pt2.x = cvRound(x0 - 1000*(-b));
        	pt2.y = cvRound(y0 - 1000*(a));
		int str = (i * 255) / lines.size();
        	line(all, pt1, pt2, cv::Scalar(255 - str,0,str), 3, cv::LINE_AA);
	}
	//cv::imshow("All Lines", all);

	//Find the "leading" line.
	double strongest_rho = lines[0][0];
	double strongest_theta = lines[0][1];
	//printf("Strongest theta is %f.\n", strongest_theta);

	//Find the strongest line which is reasonably perpendicular
	//to the leading line.
	int opposition_index = 0;
	while(abs(strongest_theta - lines[opposition_index][1]) < 1.25){
		opposition_index ++;
	}
	double opposition_rho = lines[opposition_index][0];
	double opposition_theta = lines[opposition_index][1];
	//printf("Opposition theta is %f.\n", lines[opposition_index][1]);

	//Calculate the intersection of the lines.
	double a_s = cos(strongest_theta);
	double b_s = sin(strongest_theta);
	double a_o = cos(opposition_theta);
	double b_o = sin(opposition_theta);

        double x0_s = a_s * strongest_rho;
	double y0_s = b_s * strongest_rho;
        double x0_o = a_o * opposition_rho;
	double y0_o = b_o * opposition_rho;

	cv::Point p1s, p2s, p1o, p2o;
        p1s.x = cvRound(x0_s + 1000 * -b_s);
        p1s.y = cvRound(y0_s + 1000 *  a_s);
        p2s.x = cvRound(x0_s - 1000 * -b_s);
        p2s.y = cvRound(y0_s - 1000 *  a_s);
        p1o.x = cvRound(x0_o + 1000 * -b_o);
        p1o.y = cvRound(y0_o + 1000 *  a_o);
        p2o.x = cvRound(x0_o - 1000 * -b_o);
        p2o.y = cvRound(y0_o - 1000 *  a_o);

	cv::Mat both = img.clone();
	cv::line(both, p1s, p2s, cv::Scalar(0,0,255), 3, cv::LINE_AA);
	cv::line(both, p1o, p2o, cv::Scalar(255,0,0), 3, cv::LINE_AA);
	//cv::imshow("Intersection Lines", both);

	double x1 = p1s.x;
	double y1 = p1s.y;
	double x2 = p2s.x;
	double y2 = p2s.y;
	double x3 = p1o.x;
	double y3 = p1o.y;
	double x4 = p2o.x;
	double y4 = p2o.y;

	double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	cv::Point output;
	output.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))/d;
	output.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))/d;

	cv::waitKey();
	cv::destroyAllWindows();
	
	return output;
	}
}
