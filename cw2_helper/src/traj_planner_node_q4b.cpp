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

            p(countMessage,0) = targetTF.transform.translation.x;
            p(countMessage,1) = targetTF.transform.translation.y;
            p(countMessage,2) = targetTF.transform.translation.z;

            p(countMessage,3) = targetTF.transform.rotation.x;
            p(countMessage,4) = targetTF.transform.rotation.y;
            p(countMessage,5) = targetTF.transform.rotation.z;
            p(countMessage,6) = targetTF.transform.rotation.w;

            p(countMessage,7) = targetTF.header.stamp.sec + targetTF.header.stamp.nsec /1e9;

            std::cout<<"\n"<<p<<std::endl;
        }
        countMessage++;

    }

    bag.close();

    return p;

}

void traj_q4b (MatrixXd checkpoint, YoubotIkine *youbot_ikine)
{
    // Do something
    // Converts the checkpoint matrix into trajectory point messages
    YoubotIkine youbot_kine;
    youbot_kine = *youbot_ikine;
    double time_init=0, time_final;
    double dt;

    int countMessage = checkpoint.rows();
    int updateSize=0;

    VectorXd sol_T(6), cur_T(7),final_T(7); // pose solution using axis angle Rodrigues
    Matrix4d sol_T_mat,cur_T_mat,final_T_mat;

    traj_points.resize(5,7); // 1st 7 columns are translation(3) and rotation(3), dt(1)

    for (int cMessage = 0; cMessage<countMessage; cMessage++){
        time_final = checkpoint(cMessage,7);
        dt = 1/(time_final - time_init);

        int totalStep = ceil(time_final - time_init);

        traj_points.conservativeResize(updateSize + totalStep , 7);

        // assign checkpoint into current T and final T between 2 pose
        if (cMessage==0){
            cur_T << 0,0,0,0,0,0,0;
            final_T=checkpoint.block(cMessage,0,1,7).transpose();
        }
        else{
            cur_T = checkpoint.block(cMessage-1,0,1,7).transpose();
            final_T=checkpoint.block(cMessage,0,1,7).transpose();
        }


        cur_T_mat = youbot_kine.vectorQuat_rotationMat(cur_T);
        final_T_mat=youbot_kine.vectorQuat_rotationMat(final_T);

        double cTime = 0;
        for (int cStep=0; cStep<totalStep; cStep++){

            sol_T_mat = cur_T_mat*((cur_T_mat.inverse()*final_T_mat).log()*cTime).exp();
            sol_T = youbot_kine.rotationMatrix_Vector(sol_T_mat);

            traj_points.block(cStep + updateSize,0,1,6) = sol_T.transpose();
            traj_points(cStep + updateSize,6) = dt;
            cTime+=dt;
            // compute last time step
            if (cMessage == 9){
                traj_points.conservativeResize(updateSize + totalStep + 1 , 7);

                sol_T_mat = cur_T_mat*((cur_T_mat.inverse()*final_T_mat).log()*cTime).exp();
                sol_T = youbot_kine.rotationMatrix_Vector(sol_T_mat);

                traj_points.block(cStep + updateSize + 1,0,1,6) = sol_T.transpose();
                traj_points(cStep + updateSize + 1,6) = dt;
            }

        }

        time_init = time_final;
        updateSize = traj_points.rows();
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

    traj_q4b(check_point_matrix,&youbot_kine);
    int countPoints=traj_points.rows();
    int cPoints = 0;
    std::cout<<countPoints<<std::endl;
    std::cout<<"New\n"<<traj_points<<std::endl;
    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout << "start " << cPoints << "\n" << std::endl;
            Matrix4d desiredPose = youbot_kine.rotationVector_Matrix(traj_points.block(cPoints, 0, 1, 6).transpose());

//            VectorXd desiredJointPosition = youbot_kine.inverse_kinematics_closed(desiredPose);
//            VectorXd desiredJointPosition = youbot_kine.inverse_kinematics_jac(traj_points.block(cPoints,0,1,6).transpose());
//            std::cout<<desiredJointPosition.transpose()<<std::endl;

            // KDL
            KDL::Frame desiredPose_KDL = youbot_kine.poseMatrix_kdlFrame(desiredPose);
            KDL::JntArray desiredJointPosition = youbot_kdl_test.inverse_kinematics_closed(desiredPose_KDL);
//
//            std::cout<<desiredJointPosition.data.transpose()<<std::endl;

            traj_pt.positions.clear();
            for (int i = 0; i < 5; i++) {
//                traj_pt.positions.push_back(desiredJointPosition(i));
                traj_pt.positions.push_back(desiredJointPosition.data(i));
            }
            double dt = traj_points(cPoints, 6);
            youbot_kine.publish_trajectory(traj_pt, dt * 1e9);

            ros::Duration(dt).sleep();
            std::cout << "after publish\n" << std::endl;

            ros::spinOnce();
        }
    }

    return 0;
}
