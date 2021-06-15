#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include <sprayhead_calibration/Sprayhead_Calibrate.h>

std::vector<cv::Point2f> points;

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
	
	int sizemeasure = std::max(
		std::max(original_image.rows, original_image.cols) / 250,
		2
	);
	cv::Mat de = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,
		cv::Size(sizemeasure, sizemeasure),
		cv::Point(sizemeasure / 2, sizemeasure / 2)
	);
	
	std::vector<double> final_lines;
	for(int n = 0; n < points.size() && ros::ok(); n++){
		uchar hue_core = hsv[0].at<uchar>(points[n]);
		uchar sat_core = hsv[1].at<uchar>(points[n]);
		uchar val_core = hsv[2].at<uchar>(points[n]);
		//ROS_INFO("%d %d %d\n", hue_core, sat_core, val_core);
		
		cv::Mat hue_diff = cv::abs(hsv[0] - hue_core);
		cv::Mat sat_diff = cv::abs(hsv[1] - sat_core);
		cv::Mat val_diff = cv::abs(hsv[2] - val_core);
		/*cv::imshow("Hue Diff", hue_diff);
		cv::imshow("Sat Diff", sat_diff);
		cv::imshow("Lum Diff", val_diff);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		//Numerical ops like sqrt don't work on uchar (!).
		cv::Mat hue_diff_d;
		cv::Mat sat_diff_d;
		cv::Mat val_diff_d;
	
		hue_diff.convertTo(hue_diff_d, CV_64FC1);
		sat_diff.convertTo(sat_diff_d, CV_64FC1);
		val_diff.convertTo(val_diff_d, CV_64FC1);
		
		cv::Mat fitness;
		cv::sqrt(//No easy ^2 function
			//hue_diff_d.mul(hue_diff_d) +//Selecting white things by hue generally produces unusable results.
			sat_diff_d.mul(sat_diff_d) +
			val_diff_d.mul(val_diff_d)
		, fitness);
		
		uchar avg_fitness = cv::mean(fitness)[0];
		fitness = fitness / 255.0;
		/*Icv::imshow("Crude Fitness", fitness);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		cv::Mat fitthresh_fl = (fitness < 0.25) * 255;
		cv::Mat fitthresh;
		fitthresh_fl.convertTo(fitthresh, CV_8UC1);
		/*cv::imshow("Thresholded Fitness", fitthresh);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		//Fill in an intermediate value since there are already a lot of 255s in an otherwise black image.
		cv::floodFill(fitthresh, points[n], cv::Scalar(128));
		cv::Mat fitthresh_windowed = (fitthresh == 128);
		/*cv::imshow("Single Point Window", fitthresh_windowed);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		cv::Mat dilated;
		cv::dilate(fitthresh_windowed, dilated, de);
		/*cv::imshow("Thresholded Expanded Fitness", dilated);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		fitness = fitness * 255.0;
		cv::Mat fitness_flipped;
		fitness.convertTo(fitness_flipped, CV_8UC1);
		fitness_flipped = (255-fitness_flipped) - (avg_fitness - 50);//Subtraction of average prevents the background from making a line where it is windowed.
		/*cv::imshow("Recalculated Fitness", fitness_flipped);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		cv::Mat windowed;
		fitness_flipped.copyTo(windowed, dilated);
		/*cv::imshow("Window of Examination", windowed);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		
		cv::Mat ow;
		cv::Canny(windowed, ow, 50, 200, 3);/*cv::imshow("Edges", ow);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		std::vector<cv::Vec2f> lines;
		HoughLines(ow, lines, 0.5, CV_PI/90.0, 15, 0, 0, 0, CV_PI);
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
		for(int i = 0; i < lines.size(); i++){//For some reason, this sometimes detects vertical lines with negative thetas
			if(abs(lines[i][1]) > (CV_PI / 2.0 - 0.01) && abs(lines[i][1]) < (CV_PI / 2.0 + 0.01) && lines[i][0] > 0){
				filtered_lines.push_back(lines[i]);
			}
		}
		/*// Draw the horizontal lines
		cv::Mat ver = original_image.clone();
		for(int i = 0; i < filtered_lines.size(); i++ ){
        		float rho = filtered_lines[i][0];
			printf("%f\n", rho);
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
		cv::imshow("Horizontal Lines", ver);
		cv::waitKey();
		cv::destroyAllWindows();
		printf("\n\n");*/
		
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
					accumulation_vector.push_back(filtered_lines[i]);
					size_vector.push_back(1);
					anchor_vector.push_back(filtered_lines[i]);
				}
			}
			
			decimated_lines = std::vector<cv::Vec2f>(accumulation_vector.size());
			for(int i = 0; i < decimated_lines.size(); i++){
				decimated_lines[i] = accumulation_vector[i] / size_vector[i];
			}	
				
			threshold += 0.1;
		} while(decimated_lines.size() > 2);
		
		/*// Draw the condensed lines
		cv::Mat con = original_image.clone();
		for(int i = 0; i < decimated_lines.size(); i++ ){
	        	float rho = decimated_lines[i][0];
			float theta = decimated_lines[i][1];
			printf("%f\n", rho);
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
		cv::destroyAllWindows();*/
		
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
	ros::init(argc, argv, "sprayhead_calibrator_h");
	ros::NodeHandle nh;
	
	if(argc < 2){
		ROS_ERROR("Node needs a seed-values file to run, and one was not provided. Terminating.\n");
		return 0;
	}
	
	std::ifstream data_file;
	data_file.open(argv[1]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[1]);
		return 0;
	}
	
	int n = 0;
	char line [128];
	while(data_file.getline(line, 128)){
		n++;
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 1){
			ROS_ERROR("Bad file format. Line %d (%s).\n", n, line);
			return 0;
		}
		
		cv::Point2d px;
		sscanf(line, "%lf, %lf",
			&px.x, &px.y
		);
		
		points.push_back(px);
	}
	ROS_INFO("Read in \e[1m%d\e[0m entries from %s.\n\n", n, argv[1]);
	if(n != 8){
		ROS_ERROR("There should be 8, terminating.\n");
		return 0;
	}	
	
	ros::ServiceServer cl = nh.advertiseService("/analyze_sprayhead_image_horizontal", srv_CB);
	
	ros::spin();

	return 0;
}
