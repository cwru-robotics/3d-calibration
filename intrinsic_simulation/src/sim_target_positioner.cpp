#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/DeleteModel.h>

ros::Publisher success_publisher;
void target_move_CB(const geometry_msgs::Point::ConstPtr & p){

	//Delete the old target
	gazebo_msgs::DeleteModel dm;
	dm.request.model_name = "target";
	while(!ros::service::call("/gazebo/delete_model", dm) && ! ros::ok()){
		ros::spinOnce();
	}
	ros::Duration(1.0).sleep();
	
	//Spawn a new target
	system(("roslaunch "
		+ ros::package::getPath("intrinsic_simulation")
		+ "/xml/spawn_target.launch"
		+ " x:=" + std::to_string(p->x) + " y:=" + std::to_string(p->y) + " z:=" + std::to_string(p->z)
	).c_str());
	ros::Duration(1.0).sleep();

	//Give the go-ahead to proceed
	std_msgs::Bool b;
	b.data = false;
	success_publisher.publish(b);
}

int main(int argc, char ** argv){
	//ROS initialization
	ros::init(argc, argv, "sim_placer");
	ros::NodeHandle nh;
	
	success_publisher = nh.advertise<std_msgs::Bool>("/motion_result", false);
	ros::Subscriber motion_subscriber = nh.subscribe("/motion_command", 1, target_move_CB);
	
	ROS_INFO("Simulator is ready to move target.");
	
	ros::spin();
	
	return 0;
}
