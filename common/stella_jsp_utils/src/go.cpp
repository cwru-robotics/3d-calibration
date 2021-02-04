#include <stella_jsp_utils/stella_jsp_utils.h>

int main(int argc, char ** argv){
	if(argc < 5){
		printf("Not enough args.\n");
		return 0;
	}
	
	//ROS initialization
	ros::init(argc, argv, "extrinsic_acquisition");
	ros::NodeHandle nh;
	
	//Set up publishers.
	ros::Publisher irl_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
	ros::Publisher sim_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("MH5020/arm_controller/command", 1, true);
	go::g_pub_rosi = & irl_trajectory_pub;
	go::g_pub_simu = & sim_trajectory_pub;
	
	//Warm up callbacks.
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, go::jointStatesCb);
	go::g_get_new_jspace = false;
	printf("\nWaiting for /joint_states... ");
	while(!go::g_get_new_jspace && ros::ok()){
		ros::spinOnce();
	}
	printf("Done.\n");
	
	std::vector<double> target = {std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])};
	
	go::go_to(target);

	return 0;
}
