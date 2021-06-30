#include <fstream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>

#include <xform_utils/xform_utils.h>
#include <stella_jsp_utils/stella_jsp_utils.h>

bool new_pcl;
std::string save_position;
void pcl_CB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc){
	new_pcl = true;
	if(!save_position.empty()){
		pcl::io::savePCDFileASCII(save_position, *pc);
	}
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "toolflange_acquisition");
	ros::NodeHandle nh;
	
	
	//Argumentation check.
	if(argc < 3){
		ROS_ERROR("USAGE: rosrun intrinsic_acquisition /path/to/store/images /path/for/poses.csv");
		return 0;
	}
	
	//Warm up callbacks.
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, go::jointStatesCb);
	go::g_get_new_jspace = false;
	printf("\nWaiting for /joint_states... ");
	while(!go::g_get_new_jspace && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	ros::Subscriber pc_sub = nh.subscribe("/camera/depth/points", 1, pcl_CB);
	new_pcl = false;
	save_position = "";
	printf("Waiting for /camera/depth/points... ");
	while(!new_pcl && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	printf("\n\e[1mWriting cloud output to %s.\e[0m\n", argv[1]);
	
	
	//Set up publishers.
	 ros::Publisher irl_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
	 ros::Publisher sim_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("MH5020/arm_controller/command", 1, true);
	 go::g_pub_rosi = & irl_trajectory_pub;
	 go::g_pub_simu = & sim_trajectory_pub;
	
	//Open the position output.
	std::ofstream frame_output;
	frame_output.open(argv[2]);
	if(!frame_output){
		ROS_ERROR("Error: frame output file %s could not be opened.\n", argv[2]);
		return 0;
	}
	printf("\n\e[1mWriting positional output to %s.\e[0m\n", argv[2]);
	
	//Acquire the starting pose.
	std::vector<double> goal_pose(6);
	for(int i = 0; i < 6; i++){
		goal_pose[i] = go::g_q_vec_arm_Xd[i];
	}
	
	//Move to the locations.
	printf("\nReady to move to poses.\n");
	tf::TransformListener tfl;
	tf::StampedTransform transform;
	XformUtils xfu;
	int n = 20;
	for(int i = 0; i < n; i++){
		printf("%f%%:\t", 50.0 * (float)i/(float)n);
		std::cout.flush();
		
		goal_pose[5] = ((double)i) / ((double)n) * 2.0 * M_PI;
		
		printf("Moving...\t");
		go::go_to(goal_pose);
		std::cout.flush();
		
		//Acquire image
		printf("Imaging...\t");
		std::cout.flush();
		new_pcl = false;
		save_position = std::string(argv[1]) + "pcl_" + std::to_string(i) + ".pcd";
		while(!new_pcl && ros::ok()){
			ros::spinOnce();
		}
		
		
		//Acquire transform
		printf("Acquiring transform...\t");
		std::cout.flush();
		bool got_pose = false;
		while (!got_pose && ros::ok()) {
			try {
				tfl.lookupTransform("/tool0_link", "/base_link", ros::Time(0), transform);

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
	
	goal_pose[1] += 0.01;
	goal_pose[2] -= 0.01;
	
	for(int i = 0; i < n; i++){
		printf("%f%%:\t", (50.0 * (float)i/(float)n) + 50.0);
		std::cout.flush();
		
		goal_pose[5] = ((double)i) / ((double)n) * 2.0 * M_PI;
		
		printf("Moving...\t");
		go::go_to(goal_pose);
		std::cout.flush();
		
		//Acquire image
		printf("Imaging...\t");
		std::cout.flush();
		new_pcl = false;
		save_position = std::string(argv[1]) + "pcl_" + std::to_string(i+20) + ".pcd";
		while(!new_pcl && ros::ok()){
			ros::spinOnce();
		}
		
		
		//Acquire transform
		printf("Acquiring transform...\t");
		std::cout.flush();
		bool got_pose = false;
		while (!got_pose && ros::ok()) {
			try {
				tfl.lookupTransform("/tool0_link", "/base_link", ros::Time(0), transform);

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
	
	frame_output.close();

	return 0;
}
