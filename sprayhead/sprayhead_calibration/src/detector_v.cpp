#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

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
	
	//For blue squares. This may vary.
	cv::Mat hue_diff = cv::abs(hsv[0]);//Blue appears to be hue 0 in their color system
	cv::Mat sat_diff = cv::abs(hsv[1] - 129);
	cv::Mat val_diff = cv::abs(hsv[2] - 229);
	/*cv::imshow("Hue Diff", hue_diff);
	cv::imshow("Sat Diff", sat_diff);
	cv::imshow("Lum Diff", val_diff);
	cv::waitKey();
	cv::destroyAllWindows();*/
	
	cv::Mat hue_diff_d;
	cv::Mat sat_diff_d;
	cv::Mat val_diff_d;
	
	hue_diff.convertTo(hue_diff_d, CV_64FC1);
	sat_diff.convertTo(sat_diff_d, CV_64FC1);
	val_diff.convertTo(val_diff_d, CV_64FC1);
	
	cv::Mat blueness;
	cv::sqrt(
		hue_diff_d.mul(hue_diff_d) +
		sat_diff_d.mul(sat_diff_d) +
		val_diff_d.mul(val_diff_d)
	, blueness);
	blueness = blueness / 255.0;
	cv::imshow("Crude Blueness", blueness);
	cv::waitKey();
	cv::destroyAllWindows();
	
	cv::Mat yellowthresh_fl = (blueness < 0.25) * 255;
	cv::Mat yellowthresh;
	yellowthresh_fl.convertTo(yellowthresh, CV_8UC1);
	cv::imshow("Thresholded Yellowness", yellowthresh);
	cv::waitKey();
	cv::destroyAllWindows();
	
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
	cv::Mat blueness_uc;
	blueness = 255 - (blueness * 255.0);
	blueness = blueness - 128;
	blueness.convertTo(blueness_uc, CV_8UC1);
	blueness_uc.copyTo(windowed, dilated_thresh);
	cv::imshow("Thresholded Masked Yellowness", windowed);
	cv::waitKey();
	cv::destroyAllWindows();
	
	cv::SimpleBlobDetector::Params blob_params;
	blob_params.minDistBetweenBlobs = 20;//Investigate this property
	blob_params.filterByColor = true;
	blob_params.blobColor = 255;
	
	blob_params.filterByArea = false;
	blob_params.filterByCircularity = false;
	blob_params.filterByConvexity = false;
	blob_params.filterByInertia = false;
	
	cv::Ptr<cv::SimpleBlobDetector> bd = cv::SimpleBlobDetector::create(blob_params);
	
	
	int sizemeasure = std::max(
		std::max(original_image.rows, original_image.cols) / 100,
		2
	);
	cv::Mat de = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,
		cv::Size(sizemeasure, sizemeasure),
		cv::Point(sizemeasure / 2, sizemeasure / 2)
	);
	
	cv::Mat yt_eroded;
	cv::erode(yellowthresh, yt_eroded, de);
	cv::imshow("Thresholded Eroded Yellowness", yt_eroded);
	cv::waitKey();
	cv::destroyAllWindows();
	
	std::vector<cv::KeyPoint> blob_centers;
	bd->detect(yt_eroded, blob_centers);
	
	if(blob_centers.size() != 8){
		ROS_ERROR(
			"Detected %ld yellow objects when there should be 6.\n",
			blob_centers.size()
		);
		response.success = false;
		return true;
	}
	
	std::vector<double> final_lines;
	
	for(int n = 0; n < 8; n++){
		cv::Mat yellowthresh_clone;
		yellowthresh.copyTo(yellowthresh_clone);
		for(int i = 0; i < 8; i++){
			if(i != n){
				cv::floodFill(yellowthresh_clone, blob_centers[i].pt, cv::Scalar(0));
			}
		}
		/*cv::imshow("Excluded_Blob", yellowthresh_clone);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		cv::Mat dilated_thresh;
		cv::dilate(yellowthresh_clone, dilated_thresh, de);
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
		/*cv::imshow("Edges", ow);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		std::vector<cv::Vec2f> lines;
		HoughLines(ow, lines, 0.5, CV_PI/90.0, 10, 0, 0, 0, CV_PI);
		// Draw all the lines
		/*cv::Mat all = original_image.clone();
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
			if(abs(lines[i][1]) < 0.01){
				filtered_lines.push_back(lines[i]);
			}
		}
		/*// Draw the vertical lines
		cv::Mat ver = original_image.clone();
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
        		line(ver, pt1, pt2, cv::Scalar(255 - str,0,str), 3, cv::LINE_AA);
		}
		cv::imshow("Vertical Lines", ver);
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
		} while(decimated_lines.size() != 2);
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
		
		if(decimated_lines[0][0] < decimated_lines[1][0]){
			final_lines.push_back(decimated_lines[0][0]);
			final_lines.push_back(decimated_lines[1][0]);
		} else{
			final_lines.push_back(decimated_lines[1][0]);
			final_lines.push_back(decimated_lines[0][0]);
		}	
	}
	
	printf("Changes in line distance:\n");
	for(int i = 0; i < final_lines.size(); i++){
		printf("\t%f px\n", request.guesses[i] - final_lines[i]);
		response.detections[i] = final_lines[i];
	}
	
	response.success = true;
	return true;
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "sprayhead_calibrator");
	ros::NodeHandle nh;
	
	ros::ServiceServer cl = nh.advertiseService("/analyze_sprayhead_image_vertical", srv_CB);
	
	ros::spin();

	return 0;
}
