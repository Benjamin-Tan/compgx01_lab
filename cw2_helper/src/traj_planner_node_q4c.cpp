#include <ros/ros.h>
#include <cw2_helper/YoubotIkine.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <unsupported/Eigen/MatrixFunctions>
// testing
#include <inverse_kinematics/YoubotKDL.h>

trajectory_msgs::JointTrajectoryPoint traj_pt;
MatrixXd traj_points;

//MatrixXd get_checkpoint(YoubotKDL *youbot_kine)
MatrixXd get_checkpoint(YoubotIkine *youbot_kine)
{
    rosbag::Bag bag;

    YoubotIkine youbotKine;
//    YoubotKDL youbotKine;
    youbotKine = *youbot_kine;

    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,3);

    VectorXd desired_joint_value;

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_q4c.bag", rosbag::bagmode::Read);
    topics.push_back(std::string("target_position"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::Point targetPosition;
    int countMessage=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::Point::ConstPtr j = m.instantiate<geometry_msgs::Point>();
        if (countMessage>4)
            p.conservativeResize(countMessage+1,3); //change the size of matrix without affecting old values

        if (j != NULL)
        {
            //Fill in the code to retrieve data from a bag
            targetPosition = *j;

            p(countMessage,0) = targetPosition.x;
            p(countMessage,1) = targetPosition.y;
            p(countMessage,2) = targetPosition.z;

            std::cout<<"\n"<<p<<std::endl;
        }
        countMessage++;

    }

    bag.close();

    return p;

}

void traj_q4c (MatrixXd checkpoint, YoubotIkine *youbot_ikine)
{
    // Do something
    // Converts the checkpoint matrix into trajectory point messages
    YoubotIkine youbot_kine;
    youbot_kine = *youbot_ikine;

    int countMessage = checkpoint.rows();

    double dt=0.1;
    int totalStep = 1/dt; // equals to 10 points
    int updateSize = 0;


    VectorXd sol_P(3), cur_P(3),final_P(3); // position solution
    //    Matrix4d sol_T_mat,cur_T_mat,final_T_mat;

    traj_points.resize(5,3); // 1st 3 columns are translation(3)

    for (int cMessage = 0; cMessage<countMessage; cMessage++){

        traj_points.conservativeResize(totalStep + updateSize, 3);
        // assign checkpoint into current T and final T between 2 pose
        if (cMessage==0){
            cur_P << 0,0,0;
            final_P=checkpoint.block(cMessage,0,1,3).transpose();
        }
        else{
            cur_P = checkpoint.block(cMessage-1,0,1,3).transpose();
            final_P=checkpoint.block(cMessage,0,1,3).transpose();
        }

        double cTime = 0;
        for (int cStep=0; cStep<totalStep; cStep++){
            sol_P = cur_P + cTime*(final_P-cur_P);

            traj_points.block(cStep + updateSize,0,1,3) = sol_P.transpose();

            cTime+=dt;

            //compute the last step
            if (cMessage==8){
                traj_points.conservativeResize(totalStep + updateSize + 1, 3);

                sol_P = cur_P + cTime*(final_P-cur_P);

                traj_points.block(cStep + updateSize + 1,0,1,3) = sol_P.transpose();
            }

        }

        updateSize = traj_points.rows();

    }
    std::cout<<traj_points<<std::endl;
    std::cout<<" "<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4c");

    YoubotIkine youbot_kine;

    // Remarks: both jacobian are correct magnitude, but different in signs
    // testing start ==================================================================
    YoubotKDL youbot_kdl_test;
    youbot_kdl_test.init();

/*    KDL::Jacobian trueJac = youbot_kdl_test.get_jacobian(youbot_kdl_test.current_joint_position);
//    std::cout << "True Jac\n";
//    ROS_INFO("%f %f %f %f %f\n",trueJac(0,0),trueJac(0,1),trueJac(0,2),trueJac(0,3),trueJac(0,4));
//    ROS_INFO("%f %f %f %f %f\n",trueJac(1,0),trueJac(1,1),trueJac(1,2),trueJac(1,3),trueJac(1,4));
//    ROS_INFO("%f %f %f %f %f\n",trueJac(2,0),trueJac(2,1),trueJac(2,2),trueJac(2,3),trueJac(2,4));
//    ROS_INFO("%f %f %f %f %f\n",trueJac(3,0),trueJac(3,1),trueJac(3,2),trueJac(3,3),trueJac(3,4));
//    ROS_INFO("%f %f %f %f %f\n",trueJac(4,0),trueJac(4,1),trueJac(4,2),trueJac(4,3),trueJac(4,4));
//    ROS_INFO("%f %f %f %f %f\n\n",trueJac(5,0),trueJac(5,1),trueJac(5,2),trueJac(5,3),trueJac(5,4));

    KDL::Frame current_pose = youbot_kdl_test.forward_kinematics(youbot_kdl_test.current_joint_position, youbot_kdl_test.current_pose);

    geometry_msgs::TransformStamped trans;

    trans = tf2::kdlToTransform(current_pose);
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "base_link";
    trans.child_frame_id = "arm_end_effector";

//    ROS_INFO("Translate: %f %f %f\n",trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z);
//    ROS_INFO("Rotation : %f %f %f %f\n\n",trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w);


    KDL::JntArray jointSol = youbot_kdl_test.inverse_kinematics_closed(current_pose);
    ROS_INFO("True: %f %f %f %f %f\n\n",jointSol.data(0),jointSol.data(1),jointSol.data(2),jointSol.data(3),jointSol.data(4));
    // testing end ========================================================================*/

    youbot_kine.init();

    MatrixXd check_point_matrix = get_checkpoint(&youbot_kine);

    traj_q4c(check_point_matrix,&youbot_kine);
    int countPoints=traj_points.rows();
    int cPoints = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        std::cout<<youbot_kine.obstacle_position<<std::endl;
//        for (cPoints=0;cPoints<countPoints;cPoints++) {
//            std::cout << "start " << cPoints << "\n" << std::endl;
//            Matrix4d desiredPose = youbot_kine.rotationVector_Matrix(traj_points.block(cPoints, 0, 1, 6).transpose());

//            VectorXd desiredJointPosition = youbot_kine.inverse_kinematics_closed(desiredPose);
//            VectorXd desiredJointPosition = youbot_kine.inverse_kinematics_jac(traj_points.block(cPoints,0,1,6).transpose());
//            std::cout<<desiredJointPosition.transpose()<<std::endl;

            // KDL
//            KDL::Frame desiredPose_KDL = youbot_kine.poseMatrix_kdlFrame(desiredPose);
//            KDL::JntArray desiredJointPosition = youbot_kdl_test.inverse_kinematics_closed(desiredPose_KDL);
//
//            std::cout<<desiredJointPosition.data.transpose()<<std::endl;

//            traj_pt.positions.clear();
//            for (int i = 0; i < 5; i++) {
////                traj_pt.positions.push_back(desiredJointPosition(i));
//                traj_pt.positions.push_back(desiredJointPosition.data(i));
//            }
            double dt = 2.5;//traj_points(cPoints, 6);
//            youbot_kine.publish_trajectory(traj_pt, dt * 1e9);

            ros::Duration(dt).sleep();
            std::cout << "after publish\n" << std::endl;


//        }
    }

    return 0;
}
