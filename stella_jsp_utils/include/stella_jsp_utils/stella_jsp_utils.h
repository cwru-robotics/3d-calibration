#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <trajectory_utils/TrajectoryUtils.h>

namespace go{
	TrajectoryUtils g_trajectory_utils;
	Eigen::VectorXd g_q_vec_arm_Xd(6);
	vector<int> g_arm_joint_indices;
	vector<string> g_ur_jnt_names = {
		"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t",
	};
	#define VECTOR_DIM 6 // e.g., a 6-dof vector
	const double IS_STILL_JSPACE_TOL = 0.001;
	const double MOVE_TIME = 4.0; //hard-code move time bewteen poses

	bool g_get_new_jspace = false;
	int g_ans;

	ros::Publisher *g_pub_rosi, *g_pub_simu;

	typedef vector <double> record_t;
	typedef vector <record_t> data_t;


	void map_arm_joint_indices(vector<string> joint_names);

	void jointStatesCb(const sensor_msgs::JointState &js_msg);

	bool is_still(double tol);

	void update_and_publish_traj(trajectory_msgs::JointTrajectory &des_trajectory);

	void stuff_trajectory_2point(
		const std::vector<Eigen::VectorXd> &qvecs,
		trajectory_msgs::JointTrajectory &new_trajectory
	);

	void move_to_trajectory(std::vector<Eigen::VectorXd> qvecs);
	
	void go_to(const std::vector<double> & g);
}
