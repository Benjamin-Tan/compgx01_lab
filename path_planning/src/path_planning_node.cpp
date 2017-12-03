#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <termios.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <geometry_msgs/TransformStamped.h>

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");

    ros::NodeHandle nh;
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 3);

    //TODO: Write a code to load your bag data.
    geometry_msgs::TransformStamped trans;
    rosbag::Bag bag;

    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;

    topics.push_back(std::string("desired_pose"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));


    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_waypoint;

    my_traj.joint_names.push_back("arm_joint_1");
    my_traj.joint_names.push_back("arm_joint_2");
    my_traj.joint_names.push_back("arm_joint_3");
    my_traj.joint_names.push_back("arm_joint_4");
    my_traj.joint_names.push_back("arm_joint_5");
    //TODO: Write a code to create your JointTrajectory message.
    double cPoint=1;
    foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::TransformStamped::ConstPtr t = m.instantiate<geometry_msgs::TransformStamped>();

            if (t != NULL)
            {
                trans = *t;
                trans.header.stamp = ros::Time::now();

                //KDL::Frame frame = tf2::transformToKDL(trans);
                //KDL::JntArray jointkdl = youbot.inverse_kinematics_closed(frame);

                double jointData[5]={cPoint,cPoint,-cPoint,cPoint,cPoint};
                std::cout << "Publishing joint: [";

                my_waypoint.positions.clear();

                for (int i = 0; i < 5; i++)
                {
                    my_waypoint.positions.push_back(jointData[i]);
                    std::cout << jointData[i] << "  ";
                }
                std::cout << "]" << std::endl;


                my_traj.header.stamp = ros::Time::now();
                my_waypoint.time_from_start.nsec = 537230041;

                my_traj.points.push_back(my_waypoint);

                cPoint = cPoint+0.5;
            }
        }
    bag.close();

    while (ros::ok())
    {

        traj_publisher.publish(my_traj);

        ros::spinOnce();
        std::cout << "Press any button to rerun the trajectory." << std::endl;
        int c = getch();
    }

    return 12345;
}