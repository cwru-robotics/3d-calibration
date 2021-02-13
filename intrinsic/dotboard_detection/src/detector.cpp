#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "../include/circle_detector.hpp"

//TODO This is used in multiple locations. Should it be moved into its own library?
//TODO It is also not especially efficient.
std::string replaceChar(std::string str, char ch1, char ch2) {
	for (int i = 0; i < str.length(); ++i) {
		if (str[i] == ch1) str[i] = ch2;
	}
	return str;
}

int main(int argc, char** argv) {
	if(argc < 3){
		printf("\e[33mUsage: rosrun dotboard_detection dots_detector /path/to/TDF.yml /path/to/output.csv\e[39m\n");
		return 0;
	}
	
	//Load in parameters.
	YAML::Node data_file;
	try{
		data_file = YAML::LoadFile(argv[1]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[31mTDF \"%s\" does not exist or contains syntax errors.\e[39m\n", argv[1]);
		return 0;
	}
	
	if(!data_file["target_x"]){
		printf("\e[31mTDF \"%s\" is missing its target_x field.\e[39m\n", argv[1]);
		return 0;
	}
	int target_x = data_file["target_x"].as<int>();
	if(!data_file["target_y"]){
		printf("\e[31mTDF \"%s\" is missing its target_y field.\e[39m\n", argv[1]);
		return 0;
	}
	double target_y = data_file["target_y"].as<int>();
	if(!data_file["target_spacing"]){
		printf("\e[31mTDF \"%s\" is missing its target_spacing field.\e[39m\n", argv[1]);
		return 0;
	}
	double target_spacing = data_file["target_spacing"].as<double>();
	
	if(!data_file["sled_x"]){
		printf("\e[31mTDF \"%s\" is missing its sled_x axis field.\e[39m\n", argv[1]);
		return 0;
	}
	std::string sled_x = data_file["sled_x"].as<std::string>();
	if(!data_file["sled_y"]){
		printf("\e[31mTDF \"%s\" is missing its sled_y axis field.\e[39m\n", argv[1]);
		return 0;
	}
	std::string sled_y = data_file["sled_y"].as<std::string>();
	if(!data_file["sled_z"]){
		printf("\e[31mTDF \"%s\" is missing its sled_z axis field.\e[39m\n", argv[1]);
		return 0;
	}
	std::string sled_z = data_file["sled_z"].as<std::string>();
	
	if(!data_file["circles"]){
		printf("\e[31mTDF \"%s\" is missing its circles versus checks field.\e[39m\n", argv[1]);
		return 0;
	}
	bool circles = data_file["circles"].as<bool>();
	
	if(
		sled_x.length() != 2 ||
		(sled_x[0] != 'x' && sled_x[0] != 'y' && sled_x[0] != 'z') ||
		(sled_x[1] != '+' && sled_x[1] != '-')
	){
		printf("\e[31mImparisble sled_x value \"%s\".\e[39m\n", sled_x.c_str());
		return 0;
	}
	if(
		sled_y.length() != 2 ||
		(sled_y[0] != 'x' && sled_y[0] != 'y' && sled_y[0] != 'z') ||
		(sled_y[1] != '+' && sled_y[1] != '-')
	){
		printf("\e[31mImparisble sled_y value \"%s\".\e[39m\n", sled_y.c_str());
		return 0;
	}
	if(
		sled_y.length() != 2 ||
		(sled_y[0] != 'x' && sled_y[0] != 'y' && sled_y[0] != 'z') ||
		(sled_y[1] != '+' && sled_y[1] != '-')
	){
		printf("\e[31mImparisble sled_y value \"%s\".\e[39m\n", sled_y.c_str());
		return 0;
	}
	
	//Calculate the axial mapping.
	
	//Read in images.
	std::vector<cv::Mat> vec_of_images;
	std::vector<std::string> vec_of_image_names;
	std::string folder = std::string(argv[1]);
	folder = folder.substr(0, folder.find_last_of("/\\"));
	printf("\nLooking for images in %s.\n", folder.c_str());
	
	boost::filesystem::path image_folder_path = boost::filesystem::path(folder);
	for(
		boost::filesystem::directory_entry& x
		:
		boost::filesystem::directory_iterator(image_folder_path)
	){
		if(x.path().native().find("img") != std::string::npos){
			vec_of_image_names.push_back(x.path().native());
			cv::Mat im = cv::imread(x.path().native(), CV_LOAD_IMAGE_COLOR);
			vec_of_images.push_back(im);
		}
	}
	int n_images = vec_of_image_names.size();
	printf("Loaded %d images.\n", n_images);
	if(n_images < 1){
		return 0;
	}
	
	//TODO Does this actually do anything that the above check does not?
	for(int i = 0; i < n_images; i++){
		cv::Mat example_image = vec_of_images[i];
		if(! example_image.data ){// Check for invalid input
			printf("\e[31mCould not process image %s.\e[39m\n", vec_of_image_names[i].c_str());
			return 0;
		}
	}
	int width = vec_of_images[0].cols;
	int height = vec_of_images[0].rows;
	printf("Image Width: %d\n", width);
	printf("Image Height: %d\n\n", height);
	
	//Open output file.
	std::ofstream calib_output_file;
	calib_output_file.open(argv[2], std::ofstream::trunc);
	if(!calib_output_file.is_open()){
		printf("\e[31mCould not open output file %s.\e[39m\n", argv[2]);
		return 0;
	}
	printf("Ready to write images to output file %s.\n\n", argv[2]);
		
	//Set up circle-detection loop.
	cv::Size patternsize(target_x, target_y);
	std::vector<cv::Point2f> centers; //this will be filled by the detected centers

	cv::CircleDetector circleDetector;
	cv::Ptr<cv::CircleDetector> circle_detector_ptr_ = cv::CircleDetector::create();
	
	cv::namedWindow("Src image");
	
	int failures=0;
	for (int i_image = 0; i_image < n_images; i_image++){
		cv::Mat image;
		cv::Mat grayscaleImage;
		
		image = vec_of_images[i_image];
		cv::cvtColor(image, grayscaleImage, CV_BGR2GRAY);
		bool patternfound;
		if(circles){
			patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, 1, circle_detector_ptr_);
		} else{
			patternfound = cv::findChessboardCorners(grayscaleImage, patternsize, centers);
		}
		
		if (patternfound) {
			printf("\tPattern found in image %d.\n", i_image + 1);
			
			//Find the coordinates in the mill frame.
			std::string imname = vec_of_image_names[i_image];
			imname = imname.substr(imname.find_last_of("\\/") + 1);
			imname = imname.substr(0, imname.find_first_of("."));
			imname = replaceChar(imname, 'p', '.');
			imname = imname.substr(imname.find_first_of("_") + 1);
			
			//TODO Replace with sscanf?
			double sled_x_val = std::stod(imname.substr(0, imname.find_first_of("_")));
			imname = imname.substr(imname.find_first_of("_") + 1);
			double sled_y_val = std::stod(imname.substr(0, imname.find_first_of("_")));
			imname = imname.substr(imname.find_first_of("_") + 1);
			double sled_z_val = std::stod(imname.substr(0, imname.find_first_of("_")));
			
			double camera_x, camera_y, camera_z;
			double s;
			
			if(sled_x[1] == '-'){
				s = -1.0;
			} else{
				s = 1.0;
			}
			if(sled_x[0] == 'x'){
				camera_x = sled_x_val * s;
			} else if(sled_x[0] == 'y'){
				camera_y = sled_x_val * s;
			} else {
				camera_z = sled_x_val * s;
			}
			
			if(sled_y[1] == '-'){
				s = -1.0;
			} else{
				s = 1.0;
			}
			if(sled_y[0] == 'x'){
				camera_x = sled_y_val * s;
			} else if(sled_y[0] == 'y'){
				camera_y = sled_y_val * s;
			} else {
				camera_z = sled_y_val * s;
			}
			
			if(sled_z[1] == '-'){
				s = -1.0;
			} else{
				s = 1.0;
			}
			if(sled_z[0] == 'x'){
				camera_x = sled_z_val * s;
			} else if(sled_z[0] == 'y'){
				camera_y = sled_z_val * s;
			} else {
				camera_z = sled_z_val * s;
			}
			
			//TODO Add some kind of check to make sure axes are sane??
			
			//For each point...
			if(circles){
				cv::Point2f center;
				for (int i_circle = 0; i_circle < target_x; i_circle++){
					for (int j_circle = 0; j_circle < target_y; j_circle++) {
						//Draw on the image
						int n_circle = i_circle + j_circle*target_y;
						center = centers[n_circle];
						cv::circle( image, center, 2, cv::Scalar(0,0,255), 2);
				
						//Write the data.
						//FORMAT: u, v, target_x, target_y, camera_x, camera_y, camera_z
						calib_output_file <<
							center.x << ", " << center.y << ", " <<
							i_circle * target_spacing << ", " << j_circle * target_spacing << ", " <<
							camera_x << ", " << camera_y << ", " << camera_z << "\n"
						;
					}
				}
			} else{
				cv::drawChessboardCorners(image, patternsize, centers, true);
				cv::Point2f center;
				for (int i_circle = 0; i_circle < target_y; i_circle++){
					for (int j_circle = 0; j_circle < target_x; j_circle++) {
						//Draw on the image
						int n_circle = i_circle*target_x + j_circle;
						center = centers[n_circle];
						cv::circle( image, center, 2, cv::Scalar(0,0,255), 2);
				
						//Write the data.
						//FORMAT: u, v, target_x, target_y, camera_x, camera_y, camera_z
						calib_output_file <<
							center.x << ", " << center.y << ", " <<
							i_circle * target_spacing << ", " << j_circle * target_spacing << ", " <<
							camera_x << ", " << camera_y << ", " << camera_z << "\n"
						;
					}
				}
			}
			
		} else{
			printf("\t\e[33mPattern not found in image %d.\e[39m\n", i_image);
			failures++;
		}
		
		cv::imshow("Src image", image);
		///imwrite(data_path + "red_cirlces.png", image);
		cv::waitKey(100);
	}
	
	printf(
		"\n\e[36mDetection process complete. \e[1m%d / %d\e[39m\e[36m patterns not found (\e[1m%f%%\e[36m).\e[39m\n\n",
		failures, n_images, (float)failures/(float)n_images
	);
	calib_output_file.close();
	
	return 0;
}
