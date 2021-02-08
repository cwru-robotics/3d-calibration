#include <boost/filesystem.hpp>

#include <pcl_ros/point_cloud.h>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <single_corner/single_corner.h>


int main(int argc, char ** argv){
	if(argc < 3){
		printf("\e[33mUsage: rosrun toolflange_detection toolflange_detector /path/to/poses.csv /path/to/output.csv\e[39m\n");
		return 0;
	}
	
	
	//Read in images.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB> > vec_of_pcls;
	std::vector<std::string> vec_of_pcl_names;
	std::string folder = std::string(argv[1]);
	folder = folder.substr(0, folder.find_last_of("/\\"));
	printf("\nLooking for PCLs in %s.\n", folder.c_str());
	
	boost::filesystem::path image_folder_path = boost::filesystem::path(folder);
	for(
		boost::filesystem::directory_entry& x
		:
		boost::filesystem::directory_iterator(image_folder_path)
	){
		if(x.path().native().find("pcl_") != std::string::npos){
			vec_of_pcl_names.push_back(x.path().native());
			pcl::PointCloud<pcl::PointXYZRGB> p;
			pcl::io::loadPCDFile<pcl::PointXYZRGB>(x.path().native(), p);
			vec_of_pcls.push_back(p);
		}
	}
	int n_pcl = vec_of_pcl_names.size();
	printf("Loaded %d point clouds.\n", n_pcl);
	if(n_pcl < 1){
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
	printf("\nWill publish detections to %s.\n", argv[2]);
	int d = 0;
	cv::namedWindow("Projection");
	for(int i = 0; i < n_pcl; i++){
		printf("%d:\t", i);
		
		std::string pcl_name = vec_of_pcl_names[i];
		pcl_name = pcl_name.substr(pcl_name.find_last_of("\\/") + 1);
		pcl_name = pcl_name.substr(0, pcl_name.find_first_of("."));
		pcl_name = pcl_name.substr(pcl_name.find_first_of("_") + 1);
		int pcl_number = std::stod(pcl_name);
		
		
		
		//Project PCL into an image:
		
		printf("\tProjecting...");
		std::cout.flush();
		
		//My first instinct was to use 2D arrays for these lookup tables,
		//but that is apparently actually beyond what can be accessed contiguously and causes the program to crash.
		/*std::vector<int> cloud_indices[vec_of_images[i].rows][vec_of_images[i].cols];
		std::vector<int> point_indices[vec_of_images[i].rows][vec_of_images[i].cols];
		std::vector<double> contributions[vec_of_images[i].rows][vec_of_images[i].cols];*/
		
		//printf("%d by %d\n", vec_of_pcls[i].width, vec_of_pcls[i].height);
		
		std::vector<std::vector<int > > cloud_indices =
			std::vector<std::vector<int> >(vec_of_pcls[i].height);
			
		std::vector<std::vector<std::vector<int> > > point_indices =
			std::vector<std::vector<std::vector<int> > >(vec_of_pcls[i].height);
			
		
		std::vector<std::vector<std::vector<double> > > contributions =
			std::vector<std::vector<std::vector<double> > >(vec_of_pcls[i].height);
			
		for(int r = 0; r < vec_of_pcls[i].height; r++){
			cloud_indices[r] = std::vector<int>(vec_of_pcls[i].width);
			point_indices[r] = std::vector<std::vector<int> >(vec_of_pcls[i].width);
			contributions[r] = std::vector<std::vector<double> >(vec_of_pcls[i].width);
		}
		
		cv::Mat projection = cv::Mat::zeros(vec_of_pcls[i].height, vec_of_pcls[i].width, CV_8UC3);
		
		//Project the points into the lookup tables.
		for(int p = 0; p < vec_of_pcls[i].size(); p++){
			double u = (double)vec_of_pcls[i][p].x / (double)vec_of_pcls[i][p].z;
			double v = (double)vec_of_pcls[i][p].y / (double)vec_of_pcls[i][p].z;
			
			//TODO: The min and max of  u and v after division are
			// always these magic numbers. Where do these magic 
			// numbers come from?
			u = (u + 0.576901) * 0.5 * (vec_of_pcls[i].width / 0.576901);
			v = (v + 0.342352) * 0.5 * (vec_of_pcls[i].height / 0.342352);
			
			int ind_u = (int)trunc(u);
			int ind_v = (int)trunc(v);
			
			projection.at<uchar>(ind_v, ind_u, 0) = vec_of_pcls[i][p].b;
			//TODO Why do these channels not work?
			//projection.at<uchar>(ind_v, ind_u, 1) = vec_of_pcls[i][p].g;
			//projection.at<uchar>(ind_v, ind_u, 2) = vec_of_pcls[i][p].r;
			
			cloud_indices[v][u] = p;
		}

		/*cv::imshow("Projection", projection);
		cv::waitKey(10000);*/
		
		printf("\tLocating...");
		std::cout.flush();
		cv::Point2d corner_in_image_space = sc::detect_single_corner(projection);
		
		cv::drawMarker(
			projection,
			corner_in_image_space,
			cv::Scalar(0, 255, 255),//Marker color
			cv::MARKER_TILTED_CROSS, 5//Type and size.
		);
		cv::imshow("Projection", projection);
		cv::waitKey(1000);
		
		printf("\tBack-projecting...");
		std::cout.flush();
		int ind_u = (int)trunc(corner_in_image_space.x);
		int ind_v = (int)trunc(corner_in_image_space.y);
		pcl::PointXYZRGB p = vec_of_pcls[i][cloud_indices[ind_v][ind_u]];
		
		detections_out << arm_positions_vector[i] << ", ";
		detections_out << p.x << ", " << p.y << ", " << p.z << "\n";
		
		printf(" DONE\n");
		
		d++;
	}
	
	printf("\nFound %d/%d corners.\n\n", d, n_pcl);
	cv::destroyAllWindows();
	detections_out.close();
	return 0;
}
