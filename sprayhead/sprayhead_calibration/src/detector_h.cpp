#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include <sprayhead_calibration/Sprayhead_Calibrate.h>

bool srv_CB(
	sprayhead_calibration::Sprayhead_CalibrateRequest & request,
	sprayhead_calibration::Sprayhead_CalibrateResponse & response
){
	cv::Mat original_image = cv::imread(request.image_file);
	if(original_image.data == NULL){
		ROS_ERROR("Could not find image %s\n", request.image_file.c_str());
		response.success = false;
		return true;
	}
	
	cv::Mat hsv_blob;
	cv::cvtColor(original_image, hsv_blob, cv::COLOR_RGB2HSV);
	std::vector<cv::Mat> hsv;
	cv::split(hsv_blob, hsv);
	/*cv::imshow("Hue", hsv[0]);
	cv::imshow("Sat", hsv[1]);
	cv::imshow("Lum", hsv[2]);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	cv::Mat yellowness = (hsv[1].mul(hsv[2])) / 255.0;
	/*cv::imshow("Crude Yellowness", yellowness);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	cv::Mat yellowthresh = yellowness > 100.0;
	/*cv::imshow("Thresholded Yellowness", yellowthresh);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	int sizemeasure = std::max(
		std::max(original_image.rows, original_image.cols) / 50,
		2
	);
	cv::Mat de = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,
		cv::Size(sizemeasure, sizemeasure),
		cv::Point(sizemeasure / 2, sizemeasure / 2)
	);
	cv::Mat dilated_thresh;
	cv::dilate(yellowthresh, dilated_thresh, de);
	/*cv::imshow("Thresholded Expanded Yellowness", dilated_thresh);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	
	cv::Mat windowed;
	yellowness.copyTo(windowed, dilated_thresh);
	/*cv::imshow("Thresholded Masked Yellowness", windowed);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	cv::Mat ow;
	cv::Canny(windowed, ow, 50, 200, 3);
	cv::imshow("Edges", ow);
	cv::waitKey();
	cv::destroyAllWindows();
	
	std::vector<cv::Vec2f> lines;
	HoughLines(ow, lines, 0.5, CV_PI/90.0, 60, 0, 0, 0, CV_PI);
	/*// Draw all the lines
	cv::Mat all = original_image.clone();
	for(int i = 0; i < lines.size(); i++ ){
        	float rho = lines[i][0];
		float theta = lines[i][1];
        	cv::Point pt1, pt2;
        	double a = cos(theta);
        	double b = sin(theta);
        	double x0 = a*rho;
        	double y0 = b*rho;
        	pt1.x = cvRound(x0 - 10000.0 * b);
        	pt1.y = cvRound(y0 + 10000.0 * a);
        	pt2.x = cvRound(x0 + 10000.0 * b);
        	pt2.y = cvRound(y0 - 10000.0 * a);
		int str = (i * 255) / lines.size();
        	line(all, pt1, pt2, cv::Scalar(255 - str,0,str), 3, cv::LINE_AA);
	}
	cv::imshow("All Lines", all);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	std::vector<cv::Vec2f> filtered_lines;
	for(int i = 0; i < lines.size(); i++){
		if(abs(lines[i][1]) > (CV_PI / 2.0 - 0.01)){
			filtered_lines.push_back(lines[i]);
		}
	}
	/*// Draw the horizontal lines
	cv::Mat hor = original_image.clone();
	for(int i = 0; i < filtered_lines.size(); i++ ){
        	float rho = filtered_lines[i][0];
		float theta = filtered_lines[i][1];
        	cv::Point pt1, pt2;
        	double a = cos(theta);
        	double b = sin(theta);
        	double x0 = a*rho;
        	double y0 = b*rho;
        	pt1.x = cvRound(x0 - 10000.0 * b);
        	pt1.y = cvRound(y0 + 10000.0 * a);
        	pt2.x = cvRound(x0 + 10000.0 * b);
        	pt2.y = cvRound(y0 - 10000.0 * a);
		int str = (i * 255) / filtered_lines.size();
        	line(hor, pt1, pt2, cv::Scalar(255 - str,0,str), 3, cv::LINE_AA);
	}
	cv::imshow("Horizontal Lines", hor);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	std::vector<cv::Vec2f> decimated_lines;
	double threshold = 0.1;
	do{
		std::vector<cv::Vec2f> accumulation_vector;
		std::vector<int> size_vector;
		std::vector<cv::Vec2f> anchor_vector;
		
		for(int i = 0; i < filtered_lines.size(); i++){
			bool new_anchor = true;
			for(int j = 0; j < anchor_vector.size(); j++){
				if(abs(filtered_lines[i][0] - anchor_vector[j][0]) < threshold){
					accumulation_vector[j] += filtered_lines[i];
					size_vector[j] ++;
					new_anchor = false;
					break;
				}
			}
			if(new_anchor){
				accumulation_vector.push_back(cv::Vec2f(0, 0));
				size_vector.push_back(0);
				anchor_vector.push_back(filtered_lines[i]);
			}
		}
		
		decimated_lines = std::vector<cv::Vec2f>(accumulation_vector.size());
		for(int i = 0; i < decimated_lines.size(); i++){
			decimated_lines[i] = accumulation_vector[i] / size_vector[i];
		}	
			
		threshold += 0.1;
	} while(decimated_lines.size() != 12);
	// Draw the condensed lines
	cv::Mat con = original_image.clone();
	for(int i = 0; i < decimated_lines.size(); i++ ){
        	float rho = decimated_lines[i][0];
		float theta = decimated_lines[i][1];
        	cv::Point pt1, pt2;
        	double a = cos(theta);
        	double b = sin(theta);
        	double x0 = a*rho;
        	double y0 = b*rho;
        	pt1.x = cvRound(x0 - 10000.0 * b);
        	pt1.y = cvRound(y0 + 10000.0 * a);
        	pt2.x = cvRound(x0 + 10000.0 * b);
        	pt2.y = cvRound(y0 - 10000.0 * a);
		int str = (i * 255) / decimated_lines.size();
        	line(con, pt1, pt2, cv::Scalar(255 - str,0,str), 1, cv::LINE_AA);
	}
	cv::imshow("Condensed Lines", con);
	cv::waitKey();
	cv::destroyAllWindows();
	
	std::vector<double> heights(decimated_lines.size());
	for(int i = 0; i < decimated_lines.size(); i++ ){
		heights[i] = decimated_lines[i][0];
	}
	std::sort(heights.begin(), heights.end());
	// Draw the sorted lines
	cv::Mat sorted = original_image.clone();
	for(int i = 0; i < decimated_lines.size(); i++ ){
        	float rho = heights[i];
		float theta = 1.57;
        	cv::Point pt1, pt2;
        	double a = cos(theta);
        	double b = sin(theta);
        	double x0 = a*rho;
        	double y0 = b*rho;
        	pt1.x = cvRound(x0 - 10000.0 * b);
        	pt1.y = cvRound(y0 + 10000.0 * a);
        	pt2.x = cvRound(x0 + 10000.0 * b);
        	pt2.y = cvRound(y0 - 10000.0 * a);
		int str = (i * 255) / heights.size();
        	line(sorted, pt1, pt2, cv::Scalar(255 - str,0,str), 1, cv::LINE_AA);
	}
	cv::imshow("Sorted Condensed Lines", sorted);
	cv::waitKey();
	cv::destroyAllWindows();
	
	printf("Changes in line height:\n");
	for(int i = 0; i < heights.size(); i++){
		printf("\t%f px\n", request.guesses[i] - heights[i]);
		response.detections[i] = heights[i];
	}
	
	response.success = true;
	return true;
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "sprayhead_calibrator_h");
	ros::NodeHandle nh;
	
	ros::ServiceServer cl = nh.advertiseService("/analyze_sprayhead_image_horizontal", srv_CB);
	
	ros::spin();

	return 0;
}
