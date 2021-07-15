#include <boost/filesystem.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <cc_utils/cc_utils.h>

bool find_aruco(
	const	cv::Mat & source_img,
		cv::Mat & debug_img,
	const	int marker_number,
		std::vector<cv::Point2f> & output
){
	//TODO This is not the fastest thing.
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

	std::vector<int> aruco_ids;
	std::vector<std::vector<cv::Point2f>> aruco_corners, aruco_rejects;
	cv::aruco::detectMarkers(source_img, aruco_dict, aruco_corners, aruco_ids, aruco_params, aruco_rejects);
	
	int our_id = -1;
	for(int i = 0; i < aruco_ids.size(); i++){
		if(aruco_ids[i] == marker_number){
			our_id = i;
			break;
		}
	}
	if(our_id == -1){
		return false;
	}
	
	cv::Mat colormode;
	cv::cvtColor(source_img, colormode, cv::COLOR_RGB2GRAY);
	
	cv::TermCriteria tc(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1.0);
	cv::cornerSubPix(colormode, aruco_corners[our_id], cv::Size(5, 5), cv::Size(-1, -1), tc);
	
	for(int i = 0; i < 4; i++){//The system always returns four corners 
	
	
		//Corners go clockwise from the top left
		cv::circle(debug_img, aruco_corners[our_id][i], 2, cv::Scalar(0,0,255), 2);
		
		output.push_back(aruco_corners[our_id][i]);
	}

	return true;
}


int main(int argc, char ** argv){
	if(argc < 4){
		printf("\e[33mUsage: rosrun lidar_aruco lidar_aruco /path/to/poses.csv /path/to/output.csv /path/to/camera_info.yml \e[39m\n");
		return 0;
	}
	
	
	//Read in the camera intrinsics
	YAML::Node intrinsic_file;
	try{
		intrinsic_file = YAML::LoadFile(argv[3]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mCamera intrinsics file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[3]);
		return 0;
	}
	double fx, fy, cx, cy;
	//Rectified images so no need for distortion.
	try{
		fx = intrinsic_file["fx"].as<double>();
		fy = intrinsic_file["fy"].as<double>();
		cx = intrinsic_file["cx"].as<double>();
		cy = intrinsic_file["cy"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[31mIntrinsic parse exception \"%s\".\e[39m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized intrinsics from \e[35m%s\e[39m:\n", argv[3]);
	printf("\tfx:%f\n\tfy:%f\n\tcx:%f\n\tcy:%f\n\n", fx, fy, cx, cy);
	
	
	//Read in arm positions
	std::ifstream arm_positions_in;
	arm_positions_in.open(argv[1]);
	if(!arm_positions_in){
		printf("\e[33mCould not find position file %s.\e[39m\n", argv[1]);
		return 0;
	}
	
	std::vector<Eigen::Affine3d> arm_positions_vector;
	char line [256];
	while(arm_positions_in.getline(line, 256)){
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 15){
			printf("\e[33mProblem in position file %s.\e[39m\n", argv[1]);
			printf("\e[33mLine %lu (%s).\e[39m\n", arm_positions_vector.size() + 1, line);
			return 0;
		}
		
		double dform [16];
		sscanf(line_s.c_str(), "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
			&dform[0 ], &dform[1 ], &dform[2 ], &dform[3 ],
			&dform[4 ], &dform[5 ], &dform[6 ], &dform[7 ],
			&dform[8 ], &dform[9 ], &dform[10], &dform[11],
			&dform[12], &dform[13], &dform[14], &dform[15]
		);
		
		Eigen::Affine3d a;
		Eigen::Matrix4d m;
		m <<
			dform[0 ], dform[1 ], dform[2 ], dform[3 ],
			dform[4 ], dform[5 ], dform[6 ], dform[7 ],
			dform[8 ], dform[9 ], dform[10], dform[11],
			dform[12], dform[13], dform[14], dform[15]
		;
		a.matrix() = m;
		
		arm_positions_vector.push_back(a);
		
	}
	printf("Loaded %lu arm positions from \e[35m%s\e[39m.\n", arm_positions_vector.size(), argv[1]);
	
	
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
		if(x.path().native().find("img_") != std::string::npos){
			vec_of_image_names.push_back(x.path().native());
			cv::Mat im = cv::imread(x.path().native(), cv::IMREAD_COLOR);
			vec_of_images.push_back(im);
		}
	}
	int n_images = vec_of_image_names.size();
	printf("Loaded %d images.\n", n_images);
	if(n_images < 1){
		return 0;
	}


	//Initialize output file.	
	std::ofstream detections_out;
	detections_out.open(argv[2]);
	printf("\nWill publish detections to %s.\n\n", argv[2]);
	
	cv::namedWindow("Visual Detections");
	for(int i = 0; i < n_images; i++){
		std::string image_name = vec_of_image_names[i].substr(vec_of_image_names[i].find_last_of("/\\"));
		image_name = image_name.substr(1, image_name.find_first_of(".") - 1);
		image_name = image_name.substr(image_name.find_first_of("_") + 1);
		//Need to rediscover this because images might not be stored in 0 to n numbering:
		std::string image_number = image_name.substr(0, image_name.find_first_of("_"));
		int image_pose_number = std::stoi(image_name.substr(image_name.find_first_of("_") + 1));
		
		Eigen::Affine3d image_pose = arm_positions_vector[image_pose_number];
	
		printf("Image %s:\n", image_number.c_str());
		
		
		printf("\tIdentifying visual ARUCO markers...\t");
		//std::cout.flush();
		
		cv::Mat test_img;
		vec_of_images[i].copyTo(test_img);
		std::vector<cv::Point2f> visual_dots_t;
		std::vector<cv::Point2f> visual_dots_b;
		
		//Manually invert the image to detect inverse markers
		cv::Mat inv_img = cv::Scalar(255, 255, 255) - vec_of_images[i];
		
		if(!find_aruco(inv_img, test_img, 13, visual_dots_b)){
			printf("FAILED on bottom.\n");
			continue;
		}
		if(!find_aruco(inv_img, test_img, 17, visual_dots_t)){
			printf("FAILED on top.\n");
			continue;
		}
		
		cv::imshow("Visual Detections", test_img);
		cv::waitKey(1000);
		
		printf("DONE\n");
		
		
		printf("\tReading in PCD files...\t\t\t");
		
		
		std::vector<pcl::PointCloud<pcl::PointXYZI> > vec_of_pcds;
		std::vector<int> vec_of_pcd_transform_indices;
		int max_transform_index = 0;
		int min_transform_index = INT_MAX;
		for(
			boost::filesystem::directory_entry& x
			:
			boost::filesystem::directory_iterator(image_folder_path)
		){
			if(x.path().native().find("pcd_" + image_number + "_") != std::string::npos){
				pcl::PointCloud<pcl::PointXYZI> ptmp;
				pcl::io::loadPCDFile<pcl::PointXYZI>(x.path().native(), ptmp);
				
				std::string pcd_name = x.path().native().substr(x.path().native().find_last_of("/\\"));
				pcd_name = pcd_name.substr(1, pcd_name.find_first_of(".") - 1);
				pcd_name = pcd_name.substr(pcd_name.find_first_of("_") + 1);
				int pcd_pose_number = std::stoi(pcd_name.substr(pcd_name.find_first_of("_") + 1));
				
				vec_of_pcds.push_back(ptmp);
				vec_of_pcd_transform_indices.push_back(pcd_pose_number);
				
				if(pcd_pose_number > max_transform_index){
					max_transform_index = pcd_pose_number;
				}
				if(pcd_pose_number < min_transform_index){
					min_transform_index = pcd_pose_number;
				}
			}
		}
		int n_pcds = vec_of_pcds.size();
		printf("LOADED %d\n", n_pcds);
		if(n_pcds < 1){
			return 0;
		}
		
		printf("\tNormalizing clouds...\t\t\t");
		
		//Since the forearm remains fixed in position and turns from one
		//extreme angle to another, and the positions it records are in
		//order, the center of the viewing arc will be in the center of
		//the list. This need only be approximate; all we are trying to
		//do is move everything to a position where most things will be
		//visible in the camera projection and not skewed.
		Eigen::Affine3d center_pose = arm_positions_vector[(max_transform_index + min_transform_index) / 2];
		
		std::vector<pcl::PointCloud<pcl::PointXYZI> > transformed_clouds(n_pcds);
		for(int c = 0; c < n_pcds; c++){
			//Shift everything respective to the center pose.
			Eigen::Affine3d centering_transform = center_pose * (arm_positions_vector[vec_of_pcd_transform_indices[c]]).inverse();
			pcl::transformPointCloud(vec_of_pcds[c], transformed_clouds[c], centering_transform);
			
			//A point cloud has the coordinate system "x in, y up, z left".
			//A camera has the coordinate system "z out, y down, x right".
			//We want to turn one into the other.
			for(int p = 0; p < transformed_clouds[c].size(); p++){
				double x_tmp = -transformed_clouds[c][p].z;
				double y_tmp = -transformed_clouds[c][p].y;
				double z_tmp =  transformed_clouds[c][p].x;
				
				transformed_clouds[c][p].x = x_tmp;
				transformed_clouds[c][p].y = y_tmp;
				transformed_clouds[c][p].z = z_tmp;
			}
			
			
		}
		printf("DONE\n");
		
		//Debug code: push all the combined points into one PCD and save it
		//This can then be published and viewed with PCD_utils and RVIZ.
		pcl::PointCloud<pcl::PointXYZI> mondo;
		for(int c = 0; c < n_pcds; c++){
			mondo += transformed_clouds[c];
		}
		pcl::io::savePCDFileASCII(folder + "/debug_pcd.pcd", mondo);
		
		printf("\tProjecting...\t\t\t\t");
		
		//My first instinct was to use 2D arrays for these lookup tables,
		//but that is apparently actually beyond what can be accessed contiguously and causes the program to crash.
		
		/*std::vector<int> cloud_indices[vec_of_images[i].rows][vec_of_images[i].cols];
		std::vector<int> point_indices[vec_of_images[i].rows][vec_of_images[i].cols];
		std::vector<double> contributions[vec_of_images[i].rows][vec_of_images[i].cols];*/
		
		std::vector<std::vector<std::vector<int> > > cloud_indices =
			std::vector<std::vector<std::vector<int> > >(vec_of_images[i].rows);
			
		std::vector<std::vector<std::vector<int> > > point_indices =
			std::vector<std::vector<std::vector<int> > >(vec_of_images[i].rows);
			
		
		std::vector<std::vector<std::vector<double> > > contributions =
			std::vector<std::vector<std::vector<double> > >(vec_of_images[i].rows);
			
		for(int r = 0; r < vec_of_images[i].rows; r++){
			cloud_indices[r] = std::vector<std::vector<int> >(vec_of_images[i].cols);
			point_indices[r] = std::vector<std::vector<int> >(vec_of_images[i].cols);
			contributions[r] = std::vector<std::vector<double> >(vec_of_images[i].cols);
		}
		
		cv::Mat projection = cv::Mat(vec_of_images[i].rows, vec_of_images[i].cols, CV_8UC1);
		
		//Project the points into the lookup tables.
		for(int c = 0; c < n_pcds; c++){
			for(int p = 0; p < transformed_clouds[c].size(); p++){
				double u, v;
				cc_utils::project(
					(double)transformed_clouds[c][p].x, (double)transformed_clouds[c][p].y, (double)transformed_clouds[c][p].z,
					fx, fy, cx, cy,
					0., 0., 0., 0., 0.,//We don't want any distortion
		
					(double)vec_of_images[i].cols, (double)vec_of_images[i].rows,//TODO Not used?
		
					u, v
				);
				printf("%f, %f\n", u, v);
			}
		}
		
		printf("DONE\n");
		
		
		printf("\tDetecting pointcloud ARUCOs...\t\t");
		printf("DONE\n");
		
		printf("\tRecording data...\t\t\t");
		printf("DONE\n\n");
	}
	
	
	detections_out.close();
	
	cv::destroyAllWindows();
	
	return 0;
/*
	
	
	std::ofstream detections_out;
	detections_out.open(argv[2]);
	int top_marker = 0;
	int bot_marker = 0;
	printf("\nWill publish detections to %s.\n", argv[2]);
	cv::namedWindow("Detections");
	for(int i = 0; i < n_images; i++){
		printf("%d:\t", i);
		
		std::string imname = vec_of_image_names[i];
		imname = imname.substr(imname.find_last_of("\\/") + 1);
		imname = imname.substr(0, imname.find_first_of("."));
		imname = imname.substr(imname.find_first_of("_") + 1);
		int imnum = std::stod(imname);
		
		cv::Mat demo_image;
		vec_of_images[i].copyTo(demo_image);
		
		//Manually invert the image to detect inverse markers
		cv::Mat inv_img = cv::Scalar(255, 255, 255) - vec_of_images[i];
		
		if(find_aruco(
			inv_img,
			demo_image,
			13,
			arm_positions_vector[imnum],
			detections_out
		)){
			printf("\e[92m13\e[0m\t");
			bot_marker ++;
		} else{
			printf("\e[91m13\e[0m\t");
		}
		
		if(find_aruco(
			inv_img,
			demo_image,
			17,
			arm_positions_vector[imnum],
			detections_out
		)){
			printf("\e[92m17\e[0m\n");
			top_marker ++;
		} else{
			printf("\e[91m17\e[0m\n");
		}
		
		cv::imshow("Detections", demo_image);
		cv::waitKey(1000);
	}
	
	printf("\nFound %d/%d top markers and %d/%d bottom markers.\n\n", top_marker, n_images, bot_marker, n_images);
	cv::destroyAllWindows();
	detections_out.close();
	return 0;*/
}
