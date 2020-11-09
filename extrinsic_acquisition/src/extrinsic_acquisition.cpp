#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

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

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "extrinsic_acquisition");
	ros::NodeHandle nh;
	
	
	//Argumentation check.
	if(argc < 5){
		ROS_ERROR("USAGE: rosrun intrinsic_acquisition /path/to/poses.csv /path/to/store/images /path/for/camera/info.txt /path/for/poses.csv");
		return 0;
	}
	
	
	//Read in the poses we are supposed to visit
	//We only care about the first four joints even though 6 are recorded.
	std::vector<std::vector<double> > poses_to_visit;
	
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
		double a,b;
		//TODO Figure out how to record the other half of this data set (the half that begins with negaitve signs)
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf",
			&(arm_entry[0]), &(arm_entry[1]),
			&(arm_entry[2]), &(arm_entry[3]),
			&(arm_entry[4]), &(arm_entry[5])
		);
		poses_to_visit.push_back(arm_entry);
	}
	printf("\n\e[1mRead in %d poses to visit from %s.\e[0m\n", n, argv[1]);
	
	
	//Save camera info so that we know how the images were generated.
	printf("\nLooking for topic camera/camera_info... ");
	sensor_msgs::CameraInfo ci = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/camera_info", nh));
	printf("Done.\n\e[1mWriting to %s.\e[0m\n", argv[3]);
	std::ofstream camera_of = std::ofstream();
	camera_of.open(argv[3]);
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
	
	
	//Set up publishers.
	 ros::Publisher irl_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
	 ros::Publisher sim_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("MH5020/arm_controller/command", 1, true);
	 go::g_pub_rosi = & irl_trajectory_pub;
	 go::g_pub_simu = & sim_trajectory_pub;
	
	//Check positions
	printf("\nChecking if robot is near home position... ");
	for(int i = 0; i < VECTOR_DIM; i++){
		if(abs(go::home_pose[i] - go::g_q_vec_arm_Xd[i]) > 0.1){
			printf("\n\e[31mJoint %d is not within 0.1 of home.\e[0m\n", i);
			return 0;
		}
	}
	printf("Done.\n");
	
	
	//Open the position output.
	std::ofstream frame_output;
	frame_output.open(argv[4]);
	if(!frame_output){
		ROS_ERROR("Error: frame output file %s could not be opened.\n", argv[4]);
		return 0;
	}
	printf("\n\e[1mWriting positional output to %s.\e[0m\n", argv[4]);
	
	//Move to the locations.
	printf("\nReady to move to poses.\n");
	cv::namedWindow("Acquired Image");
	tf::TransformListener tfl;
	tf::StampedTransform transform;
	XformUtils xfu;
	for(int i = 0; i < n; i++){
		printf("\t%f%%:\t", 100.0 * (float)i/(float)n);
		std::cout.flush();
		
		printf("Moving...\t");
		go::go_to(poses_to_visit[i]);
		std::cout.flush();
		
		//Acquire image
		recent_image = false;
		while(!recent_image && ros::ok()){
			ros::spinOnce();
		}
		cv::imshow("Acquired Image", most_recent_image);
		cv::waitKey(1000);
		cv::imwrite(
			std::string(argv[2]) + "img_" + std::to_string(i) + ".png",
			most_recent_image
		);
		printf("Imaging...\t");
		std::cout.flush();
		
		
		//Acquire transform
		bool got_pose = false;
		while (!got_pose && ros::ok()) {
			try {
				tfl.lookupTransform("/forearm", "/base_link", ros::Time(0), transform);

				Eigen::Affine3d a = xfu.transformTFToAffine3d(transform);
				Eigen::Matrix4d m = a.matrix();

				for(int r = 0; r < 4; r++){
					for(int c = 0; c < 4; c++){
						frame_output << std::to_string(m(r, c)) << ", ";
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
		
		
        	printf("Done.\n");
	}
	
	cv::destroyAllWindows();
	frame_output.close();
	std::vector<double> home = {
		0, 0, 0, 0, 0, 0
	};
	go::go_to(home);

	return 0;
}
