#include <inverse_kinematics/YoubotKDL.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <termios.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

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

    ros::init(argc, argv, "youbot_bag_follower");

    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped trans;
    rosbag::Bag bag;

    YoubotKDL youbot;
    youbot.init();

    trajectory_msgs::JointTrajectoryPoint joint_array;

    ////Change the name of the file to the corresponding question.
    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;

    topics.push_back(std::string("desired_pose"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::TransformStamped::ConstPtr t = m.instantiate<geometry_msgs::TransformStamped>();

        if (t != NULL)
        {
            trans = *t;
            trans.header.stamp = ros::Time::now();
            broadcaster.sendTransform(trans);

            KDL::Frame current_pose = youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose);

            youbot.broadcast_pose(current_pose);

            KDL::Frame frame = tf2::transformToKDL(trans);

            KDL::JntArray jointkdl = youbot.inverse_kinematics_closed(frame);
            std::cout << "Publishing joint: [";

            joint_array.positions.clear();

            for (int i = 0; i < 5; i++)
            {
                joint_array.positions.push_back(jointkdl.data(i));
                std::cout << jointkdl.data(i) << "  ";
            }

            std::cout << "]" << std::endl;

            ros::Duration(0.2).sleep();

            youbot.publish_joint_trajectory(joint_array);
            ros::Duration(1.0).sleep();

            ros::spinOnce();
            std::cout << "Press any button to continue the trajectory." << std::endl;
            int c = getch();

        }
    }

}
