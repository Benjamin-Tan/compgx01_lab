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
MatrixXd traj_points;

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;


    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,11); // 1st 5 columns (position), 2nd 5 columns (velocity), last column (time)

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_q4a.bag", rosbag::bagmode::Read);
    topics.push_back(std::string("joint_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::JointState jointData;
    int countMessage=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr j = m.instantiate<sensor_msgs::JointState>();
        if (countMessage>4)
            p.conservativeResize(countMessage+1,11); //change the size of matrix without affecting old values

        if (j != NULL)
        {
            //Fill in the code to retrieve data from a bag
            jointData = *j;
            for (int i=0; i<5; i++) {

                p(countMessage,i) = jointData.position[i];
                p(countMessage,i+5) = jointData.velocity[i];
            }
            // combine the time stamp into sec
            p(countMessage,10) = jointData.header.stamp.sec + jointData.header.stamp.nsec / 1e9;

        }
        countMessage++;

    }

    bag.close();

    return p;

}

MatrixXd computeA_constant(MatrixXd checkpoint, int cMessage){

    MatrixXd A;
    A.resize(5, 4);
    VectorXd a(4), q(4);

    double time_init, time_final;

    if (cMessage == 0) {
        time_init = 0;
        time_final = checkpoint(cMessage, 10);
    } else {
        time_init = checkpoint(cMessage - 1, 10);
        time_final = checkpoint(cMessage, 10);
    }

    time_final = time_final - time_init;
    time_init = 0;

    Matrix4d timeMat;
    timeMat << 1,  time_init,  pow(time_init, 2),      pow(time_init, 3),
               0,          1,      2 * time_init,  3 * pow(time_init, 2),
               1, time_final, pow(time_final, 2),     pow(time_final, 3),
               0,          1,     2 * time_final, 3 * pow(time_final, 2);

    // loop through each joints to find corresponding constant a
    for (int i = 0; i < 5; i++) {
        if (cMessage == 0) {
            q << 0, 0, checkpoint(cMessage, i), checkpoint(cMessage, i + 5);
            a = timeMat.inverse() * q;
            A.row(i) = a.transpose();
        }
        else {
            q << checkpoint(cMessage - 1, i), checkpoint(cMessage - 1, i + 5), checkpoint(cMessage,i), checkpoint(cMessage, i + 5);
            a = timeMat.inverse() * q;
            A.row(i) = a.transpose();
        }
    }

    return A;
}

void traj_q4a (MatrixXd checkpoint)
{
    // Do something
    // Converts the checkpoint matrix into trajectory point messages using cubic spline

    double time_init, time_final;
    double dt = 0.1; // 0.5 seconds

    int countMessage = checkpoint.rows();
    int updateSize = 0;

    VectorXd sol_pos(5), sol_vel(5); // solution for position and velocity
    Vector4d t_pos, t_vel; // time vector for position and velocity

    traj_points.resize(5,11);


    for (int cMessage = 0;cMessage<countMessage;cMessage++) {

        MatrixXd A = computeA_constant(checkpoint,cMessage);
        std::cout<<cMessage<<"\n"<<A<<std::endl;
        time_final = checkpoint(cMessage,10);

        int totalStep = floor((time_final-time_init) / dt);
        // Compute trajectory
        traj_points.conservativeResize(updateSize + totalStep, 11); //change the size of matrix without affecting old values

        double cTime = 0;
        for (int cStep = 0; cStep < totalStep; cStep++) {

            t_pos << 1, cTime, pow(cTime, 2),     pow(cTime, 3);
            t_vel << 0,     1,     2 * cTime, 3 * pow(cTime, 2);

            sol_pos = A * t_pos;
            sol_vel = A * t_vel;

            traj_points.block(cStep + updateSize, 0, 1, 5) = sol_pos.transpose();
            traj_points.block(cStep + updateSize, 5, 1, 5) = sol_vel.transpose();

            cTime+= dt;
            // compute the last step
            if (cMessage==9){

                traj_points.conservativeResize(updateSize + totalStep + 1, 11); //change the size of matrix without affecting old values

                t_pos << 1, cTime, pow(cTime, 2),     pow(cTime, 3);
                t_vel << 0,     1,     2 * cTime, 3 * pow(cTime, 2);

                sol_pos = A * t_pos;
                sol_vel = A * t_vel;

                traj_points.block(cStep + updateSize + 1, 0, 1, 5) = sol_pos.transpose();
                traj_points.block(cStep + updateSize + 1, 5, 1, 5) = sol_vel.transpose();
            }
        }

        time_init = time_final;
        updateSize = traj_points.rows(); // update the current rows of points

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4a");

    YoubotIkine youbot_kine;

    youbot_kine.init();

    MatrixXd check_point_matrix = get_checkpoint();

    traj_q4a(check_point_matrix);

    int countPoints=traj_points.rows();
    int cPoints = 0;

    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout<<"start "<< cPoints << std::endl;

            traj_pt.positions.clear();
            traj_pt.velocities.clear();
            // Assign the position and velocity into the message
            for (int i=0; i<5; i++){
                traj_pt.positions.push_back(traj_points(cPoints,i));
                traj_pt.velocities.push_back(traj_points(cPoints,i+5));
            }
            youbot_kine.publish_trajectory(traj_pt,1e8);

            ros::Duration(0.1).sleep();
            std::cout<<"after publish\n"<<std::endl;

            ros::spinOnce();
        }

    }

    return 0;
}
