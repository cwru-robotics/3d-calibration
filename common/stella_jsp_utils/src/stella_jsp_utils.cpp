#include <stella_jsp_utils/stella_jsp_utils.h>

namespace go{
	void map_arm_joint_indices(vector<string> joint_names) {
		// vector<string> joint_names = joint_state->name;
		// vector<string> jnt_names;

		g_arm_joint_indices.clear();
		int index;
		int n_jnts = VECTOR_DIM;
		// cout<<"num jnt names = "<<n_jnts<<endl;
		std::string j_name;
	
		for (int j = 0; j < VECTOR_DIM; j++) {
			j_name = g_ur_jnt_names[j]; // known name, in preferred order
				for (int i = 0; i < n_jnts; i++) {
					if (j_name.compare(joint_names[i]) == 0) {
						index = i;
						// cout<<"found match at index = "<<i<<endl;
						g_arm_joint_indices.push_back(index);
						break;
					}
				}
		}
		//cout << "indices of arm joints: " << endl;
		for (int i = 0; i < VECTOR_DIM; i++) {
			//cout << g_arm_joint_indices[i] << ", ";
		}
		//cout << endl;
	}
	
	
	void jointStatesCb(const sensor_msgs::JointState &js_msg) {
		// joint_states_ = js_msg; // does joint-name mapping only once
		if (g_arm_joint_indices.size() < 1) {
			int njnts = js_msg.position.size();
			//ROS_INFO("finding joint mappings for %d jnts", njnts);
			map_arm_joint_indices(js_msg.name);
		}
		for (int i = 0; i < VECTOR_DIM; i++) {
			g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
		}
		g_get_new_jspace = true;
		// cout << "CB: q_vec_arm: " << g_q_vec_arm_Xd.transpose() << endl;
	}
	
	
	bool is_still(double tol) {
		Eigen::VectorXd init_jspace(VECTOR_DIM);
		ros::spinOnce();
		init_jspace = g_q_vec_arm_Xd;
		ros::Duration(0.2).sleep();
		g_q_vec_arm_Xd[0] = 1000;
		ros::Duration(0.01).sleep();
		while (g_q_vec_arm_Xd[0] > 900) {
			ros::spinOnce();
			ros::Duration(0.002).sleep();
		}
 
		double jspace_err = (g_q_vec_arm_Xd - init_jspace).norm();
		if (jspace_err < tol) {
			return true;
		}
		return false;
	}
	
	
	void update_and_publish_traj(trajectory_msgs::JointTrajectory &des_trajectory) {
		trajectory_msgs::JointTrajectoryPoint traj_point = des_trajectory.points[0];
		trajectory_msgs::JointTrajectory simu_traj;
		ros::spinOnce(); // let callback update the sensed joint angles
		/*ROS_INFO("expected traj start: ");
		for (int i = 0; i < VECTOR_DIM; i++) {
			cout << traj_point.positions[i] << ", ";
		}
		cout << endl;*/
		ros::spinOnce();
		for (int i = 0; i < VECTOR_DIM; i++) {
			traj_point.positions[i] = g_q_vec_arm_Xd[i];
		}
		/*ROS_INFO("modified traj start: ");
			for (int i = 0; i < VECTOR_DIM; i++) {
			cout << traj_point.positions[i] << ", ";
		}
		cout << endl;*/
		des_trajectory.points[0] = traj_point;
		des_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		g_pub_rosi->publish(des_trajectory);
		//simu_traj = 
		g_trajectory_utils.strip_traj_accels(des_trajectory,simu_traj);

		g_pub_simu->publish(simu_traj);
	}
	
	
	//specialized trajectory stuffer for a path of only 2 points
	//start and end velocities = 0, and move time = fixed
	void stuff_trajectory_2point(
		const std::vector<Eigen::VectorXd> &qvecs,
		trajectory_msgs::JointTrajectory &new_trajectory
	) {
		// new_trajectory.clear();
		trajectory_msgs::JointTrajectoryPoint trajectory_point1;
		// trajectory_msgs::JointTrajectoryPoint trajectory_point2;

		trajectory_point1.positions.clear();

		new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
		new_trajectory.joint_names.clear();
		for (int i = 0; i < VECTOR_DIM; i++) {
			new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
		}

		new_trajectory.header.stamp = ros::Time::now();
		Eigen::VectorXd q_start, q_end, dqvec, qdot_vec;
		double net_time = 0.0;
		q_start = qvecs[0];
		q_end = qvecs[0];
		qdot_vec.resize(VECTOR_DIM);
		qdot_vec << 0, 0, 0, 0, 0, 0;
		//cout << "stuff_traj: start pt = " << q_start.transpose() << endl;
		//ROS_INFO("stuffing trajectory");
		// trajectory_point1.positions = qvecs[0];

		trajectory_point1.time_from_start = ros::Duration(net_time);
		for (int i = 0; i < VECTOR_DIM; i++) { // pre-sizes positions vector, so can access w/ indices later
			trajectory_point1.positions.push_back(q_start[i]);
			trajectory_point1.velocities.push_back(0.0);
		}
		new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
		// special case: next point is last point:

		double del_time = MOVE_TIME;
		//specialize for 2-pt traj:
		int iq = 1;

		//for(int iq = 1; iq < qvecs.size(); iq++) {
			q_start = q_end;
			q_end = qvecs[iq];
			dqvec = q_end - q_start;
			// cout<<"dqvec: "<<dqvec.transpose()<<endl;
			//auto del_time = transition_time(dqvec, qdot_max_vec);
			//if(del_time < dt_traj) del_time = dt_traj;
			// cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl;
			net_time += del_time;
			// ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);
			for (int i = 0; i < VECTOR_DIM; i++) { // copy over the joint-command values
				trajectory_point1.positions[i] = q_end[i];
				trajectory_point1.velocities[i] = 0.0; //dqvec[i] / del_time;
			}
			// trajectory_point1.positions = q_end;
			trajectory_point1.time_from_start = ros::Duration(net_time);
			new_trajectory.points.push_back(trajectory_point1);
		//}
		// display trajectory:
		/*for (int iq = 1; iq < qvecs.size(); iq++) {
			cout << "traj pt: ";
			for (int j = 0; j < VECTOR_DIM; j++) {
				cout << new_trajectory.points[iq].positions[j] << ", ";
			}
			cout << endl;
			cout << "arrival time: " << new_trajectory.points[iq].time_from_start.toSec() << endl;
		}*/
	}
	
	
	void move_to_trajectory(std::vector<Eigen::VectorXd> qvecs) {
		trajectory_msgs::JointTrajectory new_trajectory;
		//stuff_trajectory_2point(qvecs, new_trajectory); // convert path to traj
		g_trajectory_utils.trapezoidal_p2p_traj(qvecs[0],qvecs[1],new_trajectory);
		double wait_time = new_trajectory.points.back().time_from_start.toSec();
		g_trajectory_utils.print_traj(new_trajectory);
		update_and_publish_traj(new_trajectory);

		ros::Duration(wait_time).sleep();

		//ROS_INFO("waiting to settle...");
		while (!is_still(IS_STILL_JSPACE_TOL)) {
			ros::Duration(0.5).sleep();
		}
		//ROS_INFO("settled!");
	}
	
	void go_to(const std::vector<double> & goal){
		std::vector<Eigen::VectorXd> des_path;
		
		g_get_new_jspace=false;
		while (!g_get_new_jspace) {
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		}
		
		des_path.push_back(g_q_vec_arm_Xd);
		std::vector<double> gc(goal);
		des_path.push_back(Eigen::Map<Eigen::VectorXd>(&gc[0], goal.size()));
		//ROS_INFO("moving to pose %d",ipose);
		move_to_trajectory(des_path);
		//ROS_INFO("waiting another 2 sec for settling...");
		ros::Duration(2.0).sleep();
	}

}
