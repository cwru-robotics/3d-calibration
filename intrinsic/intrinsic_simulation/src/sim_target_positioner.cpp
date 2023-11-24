#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <turtlesim/Spawn.h>

ros::Publisher success_publisher;
ros::ServiceClient gz_client;
bool target_move_CB(turtlesim::Spawn::Request & req, turtlesim::Spawn::Response & res){

	//Delete the old target
	/*gazebo_msgs::DeleteModel dm;
	dm.request.model_name = "target";
	while(!ros::service::call("/gazebo/delete_model", dm) && ! ros::ok()){
		ros::spinOnce();
	}
	ros::Duration(1.0).sleep();
	
	//Spawn a new target
	system(("roslaunch "
		+ ros::package::getPath("intrinsic_simulation")
		+ "/xml/spawn_target.xml"
		+ " x:=" + std::to_string(req.x) + " y:=" + std::to_string(req.y) + " z:=" + std::to_string(req.theta)
	).c_str());
	ros::Duration(1.0).sleep();*/
	gazebo_msgs::SetModelState pos;
	pos.request.model_state.pose.position.x = req.x;
	pos.request.model_state.pose.position.y = req.y;
	pos.request.model_state.pose.position.z = req.theta;
	
	pos.request.model_state.model_name = "target";
	
	while(!gz_client.call(pos)){}

	//Give the go-ahead to proceed
	res.name = "";
	return true;
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "sim_placer");
	ros::NodeHandle nh;
	
	ros::ServiceServer motion_maker = nh.advertiseService("/motion_command", target_move_CB);
	gz_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	ROS_INFO("Simulator is ready to move target.");
	
	ros::spin();
	
	return 0;
}
