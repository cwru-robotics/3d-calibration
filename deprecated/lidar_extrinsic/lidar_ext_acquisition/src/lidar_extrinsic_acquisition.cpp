#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include <opencv2/highgui.hpp>

#include <xform_utils/xform_utils.h>
#include <stella_jsp_utils/stella_jsp_utils.h>

bool recent_image;
cv::Mat most_recent_image;
void CB_image(const sensor_msgs::Image::ConstPtr & im){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		most_recent_image = cv_ptr->image;
		recent_image = true;
		return;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
}

bool recent_laser;
std::string pc_file;
void CB_laser(const sensor_msgs::LaserScan::ConstPtr ls){
	if(!recent_laser){
		pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
		float ang = ls->angle_min;
		
		for(int i = 0; i < ls->ranges.size(); i++){
			pcl::PointXYZI p_tmp(ls->intensities[i]);
			
			p_tmp.x = std::sin(ang) * ls->ranges[i];
			p_tmp.y = std::cos(ang) * ls->ranges[i];
			p_tmp.z = 0.0;
			
			pcl_cloud.push_back(p_tmp);
			ang += ls->angle_increment;
		}
		if(!pc_file.empty()){
			printf("Saving pcd to %s\n", pc_file.c_str());
			pcl::io::savePCDFileASCII(pc_file, pcl_cloud);
		}
		recent_laser = true;
	}
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "lidar_extrinsic_acquisition");
	ros::NodeHandle nh;
	
	
	//Argumentation check.
	if(argc < 4){
		ROS_ERROR("USAGE: rosrun lidar_extrinsic_acquisition /path/to/poses.csv /path/to/store/images /path/for/poses.csv /path/to/save/cam/info.yml");
		return 0;
	}
	
	
	//Read in the poses we are supposed to visit
	//We only care about the first four joints even though 6 are recorded.
	std::vector<std::vector<double> > poses_to_visit;
	std::vector<double> wrist_starts;
	std::vector<double> wrist_ends;
	std::vector<double> wrist_views;
	
	std::ifstream infile;
	infile.open(argv[1]);
	if(!infile){
		ROS_ERROR("Error: file %s could not be opened.\n", argv[1]);
		return 0;
	}
	
	int n = 0;
	char line [128];
	while(infile.getline(line, 128)){
	
		n++;
		
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 5){
			printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
			return 0;
		}
		
		std::vector<double> arm_entry(6);
		double s, e, v;
		//TODO Figure out how to record the other half of this data set (the half that begins with negaitve signs)
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf",
			&(arm_entry[0]), &(arm_entry[1]),
			&(arm_entry[2]), &e,
			&s, &v
		);
		poses_to_visit.push_back(arm_entry);
		wrist_starts.push_back(s);
		wrist_ends.push_back(e);
		wrist_views.push_back(v);
	}
	printf("\n\e[1mRead in %d poses to visit from %s.\e[0m\n", n, argv[1]);
	
	
	//Save camera info so that we know how the images were generated.
	printf("\nLooking for topic camera/camera_info... ");
	sensor_msgs::CameraInfo ci = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/camera_info", nh));
	printf("Done.\n\e[1mWriting to %s.\e[0m\n", argv[4]);
	std::ofstream camera_of = std::ofstream();
	camera_of.open(argv[4]);
	camera_of << "fx: " << ci.K[0] << "\n";
	camera_of << "fy: " << ci.K[4] << "\n";
	camera_of << "cx: " << ci.K[2] << "\n";
	camera_of << "cy: " << ci.K[5] << "\n";
	camera_of << "u: " << ci.width << "\n";
	camera_of << "v: " << ci.height << "\n";
	camera_of << "k1: " << ci.D[0] << "\n";
	camera_of << "k2: " << ci.D[1] << "\n";
	camera_of << "k3: " << ci.D[4] << "\n";
	camera_of << "p1: " << ci.D[2] << "\n";
	camera_of << "p2: " << ci.D[3] << "\n";
	camera_of.close();
	
	
	//Warm up callbacks.
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, go::jointStatesCb);
	go::g_get_new_jspace = false;
	printf("\nWaiting for /joint_states... ");
	while(!go::g_get_new_jspace && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	ros::Subscriber im_sub = nh.subscribe("/camera/image_rect_color", 1, CB_image);
	recent_image = false;
	printf("Waiting for /camera/image_rect_color... ");
	while(!recent_image && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	pc_file = "";
	ros::Subscriber ls_sub = nh.subscribe("/scan", 1, CB_laser);
	recent_laser = false;
	printf("Waiting for /scan... ");
	while(!recent_laser && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	//Set up publishers.
	ros::Publisher irl_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
	ros::Publisher sim_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("MH5020/arm_controller/command", 1, true);
	go::g_pub_rosi = & irl_trajectory_pub;
	go::g_pub_simu = & sim_trajectory_pub;
	
	//Check positions
	/*printf("\nChecking if robot is near home position... ");
	for(int i = 0; i < VECTOR_DIM; i++){
		if(abs(go::home_pose[i] - go::g_q_vec_arm_Xd[i]) > 0.1){
			printf("\n\e[31mJoint %d is not within 0.1 of home.\e[0m\n", i);
			return 0;
		}
	}
	printf("Done.\n");*/
	
	
	//Open the position output.
	std::ofstream frame_output;
	frame_output.open(argv[3]);
	if(!frame_output){
		ROS_ERROR("Error: frame output file %s could not be opened.\n", argv[4]);
		return 0;
	}
	printf("\nWriting positional output to \e[1m%s\e[0m.\n", argv[3]);
	
	//Move to the locations.
	printf("\nReady to move to poses.\n");
	cv::namedWindow("Acquired Image");
	tf::TransformListener tfl;
	tf::StampedTransform transform;
	XformUtils xfu;
	int pcount = 0;
	for(int i = 0; i < n; i++){
		for(double forearm = wrist_starts[i]; forearm <= wrist_ends[i]; forearm += 0.00625){
			//Go to the position
			poses_to_visit[i][3] = forearm;
			go::go_to(poses_to_visit[i]);
			
			//Save the cloud
			pc_file = std::string(argv[2]) + "pcd_" + std::to_string(i) + "_" + std::to_string(pcount) + ".pcd";
			recent_laser = false;
			while(!recent_laser && ros::ok()){
				ros::spinOnce();
			}
			
			//Save the position
			bool got_pose = false;
			while (!got_pose && ros::ok()) {
				try {
					tfl.lookupTransform("/forearm", "/base_link", ros::Time(0), transform);

					Eigen::Affine3d a = xfu.transformTFToAffine3d(transform);
					Eigen::Matrix4d m = a.matrix();

					for(int r = 0; r < 4; r++){
						for(int c = 0; c < 4; c++){
							frame_output << std::to_string(m(r, c));
							
							if(c < 3 || r < 3){
								frame_output << ", ";
							}
						}
					}
					frame_output << "\n";
					frame_output.flush();
					got_pose = true;

					} catch (tf::TransformException ex) {
						ROS_WARN("%s", ex.what());
						ros::Duration(1.0).sleep();
						ros::spinOnce();
					}
			}
			pcount ++;
		}
		
		//Go to the viewing position
		poses_to_visit[i][3] = wrist_views[i];
		go::go_to(poses_to_visit[i]);
		recent_image = false;
		while(!recent_image && ros::ok()){
			ros::spinOnce();
		}
		cv::imshow("Acquired Image", most_recent_image);
		cv::waitKey(1000);
		cv::imwrite(
			std::string(argv[2]) + "img_" + std::to_string(i) + "_" + std::to_string(pcount) + ".png",
			most_recent_image
		);
		printf("Imaging...\t");
		std::cout.flush();
		
		
		//Acquire transform at the viewing position
		bool got_pose = false;
		while (!got_pose && ros::ok()) {
			try {
				tfl.lookupTransform("/forearm", "/base_link", ros::Time(0), transform);
				
				Eigen::Affine3d a = xfu.transformTFToAffine3d(transform);
				Eigen::Matrix4d m = a.matrix();
				for(int r = 0; r < 4; r++){
					for(int c = 0; c < 4; c++){
						frame_output << std::to_string(m(r, c));
						if(c < 3 || r < 3){
							frame_output << ", ";
						}
					}
				}
				frame_output << "\n";
				frame_output.flush();
				got_pose = true;
			} catch (tf::TransformException ex) {
				ROS_WARN("%s", ex.what());
				ros::Duration(1.0).sleep();
				ros::spinOnce();
			}
		}
		pcount ++;
	}
	
	cv::destroyAllWindows();
	frame_output.close();
	std::vector<double> home = {
		0, 0, 0, 0, 0, 0
	};
	go::go_to(home);

	return 0;
}
