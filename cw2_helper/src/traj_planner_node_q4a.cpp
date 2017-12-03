#include <ros/ros.h>
#include <cw2_helper/YoubotIkine.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// testing
#include <inverse_kinematics/YoubotKDL.h>

trajectory_msgs::JointTrajectoryPoint traj_pt;


MatrixXd get_checkpoint()
{
    rosbag::Bag bag;


    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,5);

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_q4a.bag", rosbag::bagmode::Read);
    topics.push_back(std::string("joint_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::JointState jointData;
    int countMessage=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr j = m.instantiate<sensor_msgs::JointState>();
        if (countMessage>4)
            p.conservativeResize(countMessage+1,5); //change the size of matrix without affecting old values

        if (j != NULL)
        {
            //Fill in the code to retrieve data from a bag
            jointData = *j;
            for (int i=0; i<5; i++) {
                p(countMessage,i) = jointData.position[i];
            }
            std::cout << p(countMessage,0) <<" "<<p(countMessage,1)<<" "<<p(countMessage,2)<<" "<<p(countMessage,3)<<" "<<p(countMessage,4)<<std::endl;

        }
        countMessage++;

    }

    bag.close();

    return p;

}

void traj_q4a (MatrixXd checkpoint, int countMessage)
{
    // Do something
    // Converts the checkpoint matrix into trajectory point messages

    traj_pt.positions.clear();
    for (int i=0; i<5; i++){
        traj_pt.positions.push_back(checkpoint(countMessage,i));
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4a");

    YoubotIkine youbot_kine;

    youbot_kine.init();

    MatrixXd check_point_matrix = get_checkpoint();

    int countPoints=check_point_matrix.rows();
    int cPoints = 0;
    std::cout<<countPoints<<std::endl;
    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout<<"start "<< cPoints << std::endl;

            traj_q4a(check_point_matrix,cPoints);

            ros::Duration(0.2).sleep();
            youbot_kine.publish_joint_trajectory(traj_pt);

            ros::Duration(1.5).sleep();
            std::cout<<"after publish\n"<<cPoints<<std::endl;

        }
        ros::spinOnce();
//        sleep(1);
    }

    return 0;
}
