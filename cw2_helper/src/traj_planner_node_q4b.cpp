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

//MatrixXd get_checkpoint(YoubotKDL *youbot_kine)
MatrixXd get_checkpoint(YoubotIkine *youbot_kine)
{
    rosbag::Bag bag;

    YoubotIkine youbotKine;
//    YoubotKDL youbotKine;
    youbotKine = *youbot_kine;

    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,8);

    VectorXd desired_joint_value;

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_q4b.bag", rosbag::bagmode::Read);
    topics.push_back(std::string("target_tf"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::TransformStamped targetTF;
    int countMessage=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::TransformStamped::ConstPtr j = m.instantiate<geometry_msgs::TransformStamped>();
        if (countMessage>4)
            p.conservativeResize(countMessage+1,8); //change the size of matrix without affecting old values

        if (j != NULL)
        {
            //Fill in the code to retrieve data from a bag
            targetTF = *j;

//            ROS_INFO("Translate: %f %f %f\n",targetTF.transform.translation.x,targetTF.transform.translation.y,targetTF.transform.translation.z);
//            ROS_INFO("Rotation : %f %f %f %f\n\n",targetTF.transform.rotation.x,targetTF.transform.rotation.y,targetTF.transform.rotation.z,targetTF.transform.rotation.w);

//            MatrixXd targetTF_matrix = youbotKine.pose_rotationMat(targetTF);
//            desired_joint_value = youbotKine.inverse_kinematics_jac(targetTF_matrix);
////            desired_joint_value = youbotKine.inverse_kinematics_closed(targetTF_matrix);

//            KDL::Frame frame = tf2::transformToKDL(targetTF);
//            KDL::JntArray jointkdl = youbotKine.inverse_kinematics_closed(frame);
            p(countMessage,0) = targetTF.transform.translation.x;
            p(countMessage,1) = targetTF.transform.translation.y;
            p(countMessage,2) = targetTF.transform.translation.z;

            p(countMessage,3) = targetTF.transform.rotation.x;
            p(countMessage,4) = targetTF.transform.rotation.y;
            p(countMessage,5) = targetTF.transform.rotation.z;
            p(countMessage,6) = targetTF.transform.rotation.w;

            p(countMessage,7) = targetTF.header.stamp.sec + targetTF.header.stamp.nsec /1e9;

//            for (int i=0; i<5; i++) {
//                p(countMessage,i) = desired_joint_value(i);
////                p(countMessage,i) = jointkdl.data(i);
//            }
            std::cout<<"\n"<<p<<std::endl;
        }
        countMessage++;

    }

    bag.close();

    return p;

}

void traj_q4b (MatrixXd checkpoint, int countMessage)
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
    ros::init(argc, argv, "youbot_traj_4b");

    YoubotIkine youbot_kine;
    // Remarks: both jacobian are correct magnitude, but different in signs
    // testing start ==================================================================
    YoubotKDL youbot_kdl_test;
    youbot_kdl_test.init();

    KDL::Jacobian trueJac = youbot_kdl_test.get_jacobian(youbot_kdl_test.current_joint_position);
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
    // testing end ========================================================================

    youbot_kine.init();
//    MatrixXd check_point_matrix = get_checkpoint(&youbot_kdl_test);
    MatrixXd check_point_matrix = get_checkpoint(&youbot_kine);

	MatrixXd jacobianMatrix = youbot_kine.get_jacobian(youbot_kine.current_joint_position); //validated
//    std::cout << "My Jac\n";
//    ROS_INFO("%f %f %f %f %f\n",jacobianMatrix(0,0),jacobianMatrix(0,1),jacobianMatrix(0,2),jacobianMatrix(0,3),jacobianMatrix(0,4));
//    ROS_INFO("%f %f %f %f %f\n",jacobianMatrix(1,0),jacobianMatrix(1,1),jacobianMatrix(1,2),jacobianMatrix(1,3),jacobianMatrix(1,4));
//    ROS_INFO("%f %f %f %f %f\n",jacobianMatrix(2,0),jacobianMatrix(2,1),jacobianMatrix(2,2),jacobianMatrix(2,3),jacobianMatrix(2,4));
//    ROS_INFO("%f %f %f %f %f\n",jacobianMatrix(3,0),jacobianMatrix(3,1),jacobianMatrix(3,2),jacobianMatrix(3,3),jacobianMatrix(3,4));
//    ROS_INFO("%f %f %f %f %f\n",jacobianMatrix(4,0),jacobianMatrix(4,1),jacobianMatrix(4,2),jacobianMatrix(4,3),jacobianMatrix(4,4));
//    ROS_INFO("%f %f %f %f %f\n\n",jacobianMatrix(5,0),jacobianMatrix(5,1),jacobianMatrix(5,2),jacobianMatrix(5,3),jacobianMatrix(5,4));

    MatrixXd transformMat = youbot_kine.forward_kinematics(youbot_kine.current_joint_position,5); //validated
//    std::cout<< "\nMy own forward kinematics\n";
//    ROS_INFO("%f %f %f %f\n",transformMat(0,0),transformMat(0,1),transformMat(0,2),transformMat(0,3));
//    ROS_INFO("%f %f %f %f\n",transformMat(1,0),transformMat(1,1),transformMat(1,2),transformMat(1,3));
//    ROS_INFO("%f %f %f %f\n",transformMat(2,0),transformMat(2,1),transformMat(2,2),transformMat(2,3));

    Matrix4d desired_pose = youbot_kine.pose_rotationMat(trans); //validated
//
//    std::cout<< "\ntransformation matrix from pose\n";
//    ROS_INFO("%f %f %f %f\n",desired_pose(0,0),desired_pose(0,1),desired_pose(0,2),desired_pose(0,3));
//    ROS_INFO("%f %f %f %f\n",desired_pose(1,0),desired_pose(1,1),desired_pose(1,2),desired_pose(1,3));
//    ROS_INFO("%f %f %f %f\n\n",desired_pose(2,0),desired_pose(2,1),desired_pose(2,2),desired_pose(2,3));

    VectorXd diySol = youbot_kine.inverse_kinematics_closed(transformMat);
//    ROS_INFO("Close: %f %f %f %f %f\n\n",diySol(0),diySol(1),diySol(2),diySol(3),diySol(4));

//    VectorXd current_pose_vector = youbot_kine.pose_rotationVec(trans);

    VectorXd iterSol = youbot_kine.inverse_kinematics_jac(transformMat);
//    ROS_INFO("Jac_sol: %f %f %f %f %f\n\n",iterSol(0),iterSol(1),iterSol(2),iterSol(3),iterSol(4));

    std::cout<< youbot_kdl_test.current_joint_position.data(0) <<" "<<youbot_kdl_test.current_joint_position.data(1) <<" "<<youbot_kdl_test.current_joint_position.data(2) <<std::endl;
    std::cout<< youbot_kine.current_joint_position(0) <<" "<<youbot_kine.current_joint_position(1)<<" "<<youbot_kine.current_joint_position(2)<<std::endl;

    int countPoints=check_point_matrix.rows();
    int cPoints = 0;
    std::cout<<countPoints<<std::endl;
    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout<<"start "<< cPoints << std::endl;

            traj_q4b(check_point_matrix,cPoints);

            ros::Duration(0.5).sleep();
            youbot_kine.publish_joint_trajectory(traj_pt);

            ros::Duration(2.0).sleep();
            std::cout<<"after publish\n"<<cPoints<<std::endl;

        }
        cPoints = 0;
        ros::spinOnce();
        sleep(1);
    }

    return 0;
}
