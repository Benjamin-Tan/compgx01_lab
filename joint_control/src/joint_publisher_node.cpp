#include <ros/ros.h>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "joint_publisher");
	ros::NodeHandle nh;
	
	// getting parameters
	std::vector<std::string> joint_name_list;
	nh.getParam("/EffortJointInterface_trajectory_controller/joints",joint_name_list);
	for (int i = 0; i<joint_name_list.size(); i++)
	{
		//ROS_INFO("%s\n",joint_name_list[i].c_str());
	}
	// TODO: Write publisher
	ROS_INFO("\nWrite your publisher here!\n");

	ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command",100);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		trajectory_msgs::JointTrajectory msg;

		msg.header.stamp = ros::Time::now();

		msg.joint_names = joint_name_list;
		msg.points.resize(1);
		msg.points[0].positions.resize(5);
		msg.points[0].positions[0] = 0; // [0,5.89]
		msg.points[0].positions[1] = 1.35; // [0,2.70]
		msg.points[0].positions[2] = -4; // [-4,0]
		msg.points[0].positions[3] = 2; // [0,3.26]
		msg.points[0].positions[4] = 3; // [0,5.84]

		msg.points[0].time_from_start = ros::Duration(0.1);
		joint_pub.publish(msg);
		ROS_INFO("first");
		ros::Duration(5.0).sleep();
		
		//msg.points[0].positions.resize(5);
		msg.points[0].positions[0] = 2.5; // [0,5.89]
		msg.points[0].positions[1] = 0; // [0,2.70]
		msg.points[0].positions[2] = 0; // [-4,0]
		msg.points[0].positions[3] = 1; // [0,3.26]
		msg.points[0].positions[4] = 3; // [0,5.84]

		msg.points[0].time_from_start = ros::Duration(0.1);

		joint_pub.publish(msg);
		ROS_INFO("second");
		ros::Duration(5.0).sleep();
		ros::spinOnce();
		//loop_rate.sleep();
		
	}
	


	return 0;	
}
