#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui.hpp>

#include <xform_utils/xform_utils.h>
#include <trajectory_utils/TrajectoryUtils.h>

//TODO Are some of these utilities generally useful enough to go in their own package?
std::vector<double> home_pose = {0, 0, 0, 0, 0, 0};


std::vector<std::string> known_jnt_names = {
    "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t",
};
#define N_JOINTS 6
#define SETTLE_TOLERANCE 0.05
std::vector<int> arm_joint_indices;
bool recent_joint;
std::vector<double> most_recent_joints(N_JOINTS);
void CB_joint_states(const sensor_msgs::JointState &js) {

	if(arm_joint_indices.size() < 1){//TODO What is the purpose of this? Is not the joint mapping always consistant?
		int njnts = js.position.size();
		arm_joint_indices.clear();

		for (int j = 0; j < N_JOINTS; j++) {
			std::string j_name = known_jnt_names[j]; // known name, in preferred order
			for (int k = 0; k < N_JOINTS; k++) {
				if(j_name.compare(js.name[j]) == 0){
               				arm_joint_indices.push_back(k);
                			break;
                		}
                	}
                }
        }
        
        
	for (int i = 0; i < N_JOINTS; i++) {
		most_recent_joints[i] = js.position[arm_joint_indices[i]];
	}
	recent_joint = true;
}


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


void go(
	const Eigen::VectorXd & dt,
	const ros::Publisher & irl,
	const ros::Publisher & sim
){
	TrajectoryUtils tu;

	recent_joint = false;
	while(!recent_joint) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	std::vector<std::vector<double> > des_path;
	des_path.push_back(most_recent_joints);
	std::vector<double> goal = {
		dt(0), dt(1), dt(2), dt(3), 0.0, 0.0
	};
	des_path.push_back(goal);
		
	trajectory_msgs::JointTrajectory new_trajectory;
	tu.trapezoidal_p2p_traj(
		Eigen::Map<Eigen::VectorXd>(des_path[0].data(), N_JOINTS),
		Eigen::Map<Eigen::VectorXd>(des_path[1].data(), N_JOINTS),
		new_trajectory
	);
	double wait_time = new_trajectory.points.back().time_from_start.toSec();
	
	trajectory_msgs::JointTrajectoryPoint traj_point = new_trajectory.points[0];
	//TODO Is this section here actually at all necessary?
	ros::spinOnce(); // let callback update the sensed joint angles
	for (int j = 0; j < N_JOINTS; j++) {
		traj_point.positions[j] = most_recent_joints[j];
	}
	new_trajectory.points[0] = traj_point;
	//END SECTION
	new_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
	
	//Make the arrival times cumulative because for some reason they are not now.
	//TODO Why is that the case?
	for(int j = 1; j < new_trajectory.points.size(); j++){
		new_trajectory.points[j].time_from_start += new_trajectory.points[j-1].time_from_start;
	}
	
	/*printf("Times are as follows: ");
	for(int x = 0; x < new_trajectory.points.size(); x++){
		printf("%f ", new_trajectory.points[x].time_from_start.sec + 10e-9 * new_trajectory.points[x].time_from_start.nsec);
	}
	printf("\n");*/
	
	irl.publish(new_trajectory);
	
	trajectory_msgs::JointTrajectory simu_traj;
	tu.strip_traj_accels(new_trajectory, simu_traj);
	sim.publish(simu_traj);
	
	printf("Moving...\t");
	std::cout.flush();
	
	ros::Duration(wait_time).sleep();
	
	
	//Settle
	printf("Settling...\t");
	std::cout.flush();
	bool settled = false;
	while(!settled && ros::ok()){
		ros::spinOnce();
		std::vector<double> init_jspace = most_recent_joints;
		ros::Duration(0.2).sleep();
		recent_joint = false;
		while(!recent_joint && ros::ok()){
			ros::spinOnce();
		}
		settled = true;
		for(int j = 0; j < N_JOINTS; j++){
			if(abs(most_recent_joints[j] - init_jspace[j])){
				settled = false;
				break;
			}
		}
	}
	ros::Duration(2.0).sleep(); //wait a bit more for settling
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
	std::vector<Eigen::Vector4d> poses_to_visit;
	
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
		
		Eigen::Vector4d arm_entry;
		double a,b;
		//TODO Figure out how to record the other half of this data set (the half that begins with negaitve signs)
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf",
			&(arm_entry(0)), &(arm_entry(1)),
			&(arm_entry(2)), &(arm_entry(3)),
			&a,		 &b
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
	camera_of << "p1: " << ci.K[2] << "\n";
	camera_of << "p2: " << ci.K[3] << "\n";
	camera_of.close();
	
	
	//Warm up callbacks.
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, CB_joint_states);
	recent_joint = false;
	printf("\nWaiting for /joint_states... ");
	while(!recent_joint && ros::ok()){
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
	
	//Check positions
	printf("\nChecking if robot is near home position... ");
	for(int i = 0; i < N_JOINTS; i++){
		if(abs(home_pose[i] - most_recent_joints[i]) > 0.1){
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

		//Go
		go(poses_to_visit[i], irl_trajectory_pub, sim_trajectory_pub);
		
		
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
	Eigen::VectorXd home;
	home << 0, 0, 0, 0;
	go(home, irl_trajectory_pub, sim_trajectory_pub);

	return 0;
}
