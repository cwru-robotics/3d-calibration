#include <boost/filesystem.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

bool find_aruco(
	const	cv::Mat & source_img,
		cv::Mat & debug_img,
	const	int marker_number,
	const	std::string & arm_pose,
		std::ofstream & output
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
		
		output <<
			arm_pose << " " <<
			std::to_string(marker_number) << ", " <<
			std::to_string(i) << ", " <<
			std::to_string(aruco_corners[our_id][i].x) << ", " <<
			std::to_string(aruco_corners[our_id][i].y) << "\n"
		;
		output.flush();
	}

	return true;
}


int main(int argc, char ** argv){
	if(argc < 3){
		printf("\e[33mUsage: rosrun aruco_detection aruco_detector /path/to/poses.csv /path/to/output.csv\e[39m\n");
		return 0;
	}
	
	
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
			cv::Mat im = cv::imread(x.path().native(), CV_LOAD_IMAGE_COLOR);
			vec_of_images.push_back(im);
		}
	}
	int n_images = vec_of_image_names.size();
	printf("Loaded %d images.\n", n_images);
	if(n_images < 1){
		return 0;
	}
	
	
	//Read in arm positions
	std::ifstream arm_positions_in;
	arm_positions_in.open(argv[1]);
	if(!arm_positions_in){
		printf("\e[33mCould not find position file %s.\e[39m\n", argv[1]);
		return 0;
	}
	
	std::vector<std::string> arm_positions_vector;
	char line [256];
	while(arm_positions_in.getline(line, 256)){
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 16){
			printf("\e[33mProblem in position file %s.\e[39m\n", argv[1]);
			printf("\e[33mLine %lu (%s).\e[39m\n", arm_positions_vector.size() + 1, line);
			return 0;
		}
		arm_positions_vector.push_back(line_s);
	}
	printf("Loaded %lu arm positions.\n", arm_positions_vector.size());
	
	
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
	return 0;
}
