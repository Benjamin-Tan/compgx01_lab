#include <tf2_ros/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <inverse_kinematics/YoubotKDL.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "youbot_bag_follower");

    YoubotKDL youbot;

	YoubotKinematics youbotKinematics;
    youbot.init();
	youbotKinematics.init();

    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Write);

    while (ros::ok())
    {
		trajectory_msgs::JointTrajectoryPoint msg;
		
		msg.positions.resize(5);
		msg.positions[0] = 1+sin(ros::Time::now().toSec()); // [0,5.89]
		msg.positions[1] = 2+cos(ros::Time::now().toSec()); // [0,2.70]
		msg.positions[2] = -4; // [-4,0]
		msg.positions[3] = 2; // [0,3.26]
		msg.positions[4] = 3; // [0,5.84]

		youbotKinematics.publish_joint_trajectory(msg);

		//KDL::JntArray current_joint_position;

        KDL::Frame current_pose = youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose);
        youbot.broadcast_pose(current_pose);

        geometry_msgs::TransformStamped trans;

        trans = tf2::kdlToTransform(current_pose);
        trans.header.stamp = ros::Time::now();
        trans.header.frame_id = "base_link";
        trans.child_frame_id = "arm_end_effector";

        //bag.close();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
		// has to be rosmessage in order to write into a bag
        // temporary commented off below
        bag.write("desired_pose",ros::Time::now(),trans);
    }
    bag.close();
    ROS_INFO("TEST");
    return 1;
}
