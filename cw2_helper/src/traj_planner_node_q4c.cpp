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
MatrixXd traj_points_jointSpace;
MatrixXd traj_pts_jointSpace_avoidObstacle;
// obstacle position in matrix (row,column) => (x,y,z ; 1-12 points);
MatrixXd unitBox1_position,unitBox2_position,unitCyl0_position,unitCyl1_position;

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;

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

//            std::cout<<"\n"<<p<<std::endl;
        }
        countMessage++;
    }
    bag.close();
    return p;
}

void traj_q4c (MatrixXd checkpoint,YoubotIkine youbot_kine,YoubotKDL youbot_kdl)
{
    // Do something
    // Converts the checkpoint matrix (cartesian) into joint space messages

    int countMessage = checkpoint.rows();

    traj_points_jointSpace.resize(countMessage,5);

    for (int c=0; c<countMessage; c++){
        Matrix4d desiredPose;

        desiredPose <<  1, 0, 0, checkpoint(c, 0),
                        0, 1, 0, checkpoint(c, 1),
                        0, 0, 1, checkpoint(c, 2),
                        0, 0, 0, 1;

        KDL::Frame desiredPose_KDL = youbot_kine.poseMatrix_kdlFrame(desiredPose);
        KDL::JntArray desiredJointPosition = youbot_kdl.inverse_kinematics_closed(desiredPose_KDL);

        std::cout<<desiredPose<<"\n forward"<<std::endl;
        std::cout<<youbot_kdl.forward_kinematics(desiredJointPosition,desiredPose_KDL)<<"\n diy"<<std::endl;
        std::cout<<youbot_kine.forward_kinematics(desiredJointPosition.data,4)<<std::endl;
        //std::cout<<desiredJointPosition.data<<std::endl;

        traj_points_jointSpace.block(c,0,1,5) = desiredJointPosition.data.transpose();

    }
    std::cout<<traj_points_jointSpace<<std::endl;
    std::cout<<" "<<std::endl;

//    for (int cMessage = 0; cMessage<countMessage; cMessage++){
//
//        traj_points_cartesianSpace.conservativeResize(totalStep + updateSize, 3);
//        // assign checkpoint into current T and final T between 2 pose
//        if (cMessage==0){
//            cur_P << 0,0,0;
//            final_P=checkpoint.block(cMessage,0,1,3).transpose();
//        }
//        else{
//            cur_P = checkpoint.block(cMessage-1,0,1,3).transpose();
//            final_P=checkpoint.block(cMessage,0,1,3).transpose();
//        }
//
//        double cTime = 0;
//        for (int cStep=0; cStep<totalStep; cStep++){
//            sol_P = cur_P + cTime*(final_P-cur_P);
//
//            traj_points_cartesianSpace.block(cStep + updateSize,0,1,3) = sol_P.transpose();
//
//            cTime+=dt;
//
//            //compute the last step
//            if (cMessage==8){
//                traj_points_cartesianSpace.conservativeResize(totalStep + updateSize + 1, 3);
//
//                sol_P = cur_P + cTime*(final_P-cur_P);
//
//                traj_points_cartesianSpace.block(cStep + updateSize + 1,0,1,3) = sol_P.transpose();
//            }
//
//        }
//
//        updateSize = traj_points_cartesianSpace.rows();
//
//    }

}
void compute_obstaclePosition(YoubotIkine youbot_kine){
    /*========================== obstacle declaration (dimension) ==================================*/

    Vector3d unitBox0,unitBox1,unitBox2; // (x,y,z) vector
    Vector2d unitCyl0, unitCyl1; // (radius, length) vector
    unitBox0 << 1,1,0.069438;
    unitBox1 << 0.194033,0.077038,0.19810;
    unitBox2 << 0.167065,0.112100,0.104652;
    unitCyl0 << 0.05, 0.08;
    unitCyl1 << 0.06, 0.14;


    // unitbox0 has no effect, hence not computing
    unitBox1_position.resize(3,12);
    // -x
    unitBox1_position(0,0) = youbot_kine.obstacle_position(1,0) - unitBox1(0)/2; // x
    unitBox1_position(1,0) = youbot_kine.obstacle_position(1,1); // y
    unitBox1_position(2,0) = youbot_kine.obstacle_position(1,2); // z

    unitBox1_position(0,1) = youbot_kine.obstacle_position(1,0) - unitBox1(0)/2; // x
    unitBox1_position(1,1) = youbot_kine.obstacle_position(1,1) - unitBox1(1)/2; // y
    unitBox1_position(2,1) = youbot_kine.obstacle_position(1,2) - unitBox1(2)/2; // z

    unitBox1_position(0,2) = youbot_kine.obstacle_position(1,0) - unitBox1(0)/2; // x
    unitBox1_position(1,2) = youbot_kine.obstacle_position(1,1) - unitBox1(1)/2; // y
    unitBox1_position(2,2) = youbot_kine.obstacle_position(1,2) + unitBox1(2)/2; // z

    unitBox1_position(0,3) = youbot_kine.obstacle_position(1,0) - unitBox1(0)/2; // x
    unitBox1_position(1,3) = youbot_kine.obstacle_position(1,1) + unitBox1(1)/2; // y
    unitBox1_position(2,3) = youbot_kine.obstacle_position(1,2) - unitBox1(2)/2; // z

    unitBox1_position(0,4) = youbot_kine.obstacle_position(1,0) - unitBox1(0)/2; // x
    unitBox1_position(1,4) = youbot_kine.obstacle_position(1,1) + unitBox1(1)/2; // y
    unitBox1_position(2,4) = youbot_kine.obstacle_position(1,2) + unitBox1(2)/2; // z
    // + x
    unitBox1_position(0,5) = youbot_kine.obstacle_position(1,0) + unitBox1(0)/2; // x
    unitBox1_position(1,5) = youbot_kine.obstacle_position(1,1); // y
    unitBox1_position(2,5) = youbot_kine.obstacle_position(1,2); // z

    unitBox1_position(0,6) = youbot_kine.obstacle_position(1,0) + unitBox1(0)/2; // x
    unitBox1_position(1,6) = youbot_kine.obstacle_position(1,1) - unitBox1(1)/2; // y
    unitBox1_position(2,6) = youbot_kine.obstacle_position(1,2) - unitBox1(2)/2; // z

    unitBox1_position(0,7) = youbot_kine.obstacle_position(1,0) + unitBox1(0)/2; // x
    unitBox1_position(1,7) = youbot_kine.obstacle_position(1,1) - unitBox1(1)/2; // y
    unitBox1_position(2,7) = youbot_kine.obstacle_position(1,2) + unitBox1(2)/2; // z

    unitBox1_position(0,8) = youbot_kine.obstacle_position(1,0) + unitBox1(0)/2; // x
    unitBox1_position(1,8) = youbot_kine.obstacle_position(1,1) + unitBox1(1)/2; // y
    unitBox1_position(2,8) = youbot_kine.obstacle_position(1,2) - unitBox1(2)/2; // z

    unitBox1_position(0,9) = youbot_kine.obstacle_position(1,0) + unitBox1(0)/2; // x
    unitBox1_position(1,9) = youbot_kine.obstacle_position(1,1) + unitBox1(1)/2; // y
    unitBox1_position(2,9) = youbot_kine.obstacle_position(1,2) + unitBox1(2)/2; // z
    // +y
    unitBox1_position(0,10) = youbot_kine.obstacle_position(1,0); // x
    unitBox1_position(1,10) = youbot_kine.obstacle_position(1,1) + unitBox1(1)/2; // y
    unitBox1_position(2,10) = youbot_kine.obstacle_position(1,2); // z
    // -y
    unitBox1_position(0,11) = youbot_kine.obstacle_position(1,0); // x
    unitBox1_position(1,11) = youbot_kine.obstacle_position(1,1) - unitBox1(1)/2; // y
    unitBox1_position(2,11) = youbot_kine.obstacle_position(1,2); // z
    // unitBox2 =======================================================
    unitBox2_position.resize(3,12);
    // -x
    unitBox2_position(0,0) = youbot_kine.obstacle_position(2,0) - unitBox2(0)/2; // x
    unitBox2_position(1,0) = youbot_kine.obstacle_position(2,1); // y
    unitBox2_position(2,0) = youbot_kine.obstacle_position(2,2); // z

    unitBox2_position(0,1) = youbot_kine.obstacle_position(2,0) - unitBox2(0)/2; // x
    unitBox2_position(1,1) = youbot_kine.obstacle_position(2,1) - unitBox2(1)/2; // y
    unitBox2_position(2,1) = youbot_kine.obstacle_position(2,2) - unitBox2(2)/2; // z

    unitBox2_position(0,2) = youbot_kine.obstacle_position(2,0) - unitBox2(0)/2; // x
    unitBox2_position(1,2) = youbot_kine.obstacle_position(2,1) - unitBox2(1)/2; // y
    unitBox2_position(2,2) = youbot_kine.obstacle_position(2,2) + unitBox2(2)/2; // z

    unitBox2_position(0,3) = youbot_kine.obstacle_position(2,0) - unitBox2(0)/2; // x
    unitBox2_position(1,3) = youbot_kine.obstacle_position(2,1) + unitBox2(1)/2; // y
    unitBox2_position(2,3) = youbot_kine.obstacle_position(2,2) - unitBox2(2)/2; // z

    unitBox2_position(0,4) = youbot_kine.obstacle_position(2,0) - unitBox2(0)/2; // x
    unitBox2_position(1,4) = youbot_kine.obstacle_position(2,1) + unitBox2(1)/2; // y
    unitBox2_position(2,4) = youbot_kine.obstacle_position(2,2) + unitBox2(2)/2; // z
    // + x
    unitBox2_position(0,5) = youbot_kine.obstacle_position(2,0) + unitBox2(0)/2; // x
    unitBox2_position(1,5) = youbot_kine.obstacle_position(2,1); // y
    unitBox2_position(2,5) = youbot_kine.obstacle_position(2,2); // z

    unitBox2_position(0,6) = youbot_kine.obstacle_position(2,0) + unitBox2(0)/2; // x
    unitBox2_position(1,6) = youbot_kine.obstacle_position(2,1) - unitBox2(1)/2; // y
    unitBox2_position(2,6) = youbot_kine.obstacle_position(2,2) - unitBox2(2)/2; // z

    unitBox2_position(0,7) = youbot_kine.obstacle_position(2,0) + unitBox2(0)/2; // x
    unitBox2_position(1,7) = youbot_kine.obstacle_position(2,1) - unitBox2(1)/2; // y
    unitBox2_position(2,7) = youbot_kine.obstacle_position(2,2) + unitBox2(2)/2; // z

    unitBox2_position(0,8) = youbot_kine.obstacle_position(2,0) + unitBox2(0)/2; // x
    unitBox2_position(1,8) = youbot_kine.obstacle_position(2,1) + unitBox2(1)/2; // y
    unitBox2_position(2,8) = youbot_kine.obstacle_position(2,2) - unitBox2(2)/2; // z

    unitBox2_position(0,9) = youbot_kine.obstacle_position(2,0) + unitBox2(0)/2; // x
    unitBox2_position(1,9) = youbot_kine.obstacle_position(2,1) + unitBox2(1)/2; // y
    unitBox2_position(2,9) = youbot_kine.obstacle_position(2,2) + unitBox2(2)/2; // z
    // +y
    unitBox2_position(0,10) = youbot_kine.obstacle_position(2,0); // x
    unitBox2_position(1,10) = youbot_kine.obstacle_position(2,1) + unitBox2(1)/2; // y
    unitBox2_position(2,10) = youbot_kine.obstacle_position(2,2); // z
    // -y
    unitBox2_position(0,11) = youbot_kine.obstacle_position(2,0); // x
    unitBox2_position(1,11) = youbot_kine.obstacle_position(2,1) - unitBox2(1)/2; // y
    unitBox2_position(2,11) = youbot_kine.obstacle_position(2,2); // z
    // unitCyl0 =======================================================

    unitCyl0_position.resize(3,12);
    unitCyl0_position(0,0) = youbot_kine.obstacle_position(3,0) + unitCyl0(0); // x
    unitCyl0_position(1,0) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,0) = youbot_kine.obstacle_position(3,2); // z

    unitCyl0_position(0,1) = youbot_kine.obstacle_position(3,0) - unitCyl0(0); // x
    unitCyl0_position(1,1) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,1) = youbot_kine.obstacle_position(3,2); // z

    unitCyl0_position(0,2) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,2) = youbot_kine.obstacle_position(3,1) + unitCyl0(0); // y
    unitCyl0_position(2,2) = youbot_kine.obstacle_position(3,2); // z

    unitCyl0_position(0,3) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,3) = youbot_kine.obstacle_position(3,1) - unitCyl0(0); // y
    unitCyl0_position(2,3) = youbot_kine.obstacle_position(3,2); // z

    unitCyl0_position(0,4) = youbot_kine.obstacle_position(3,0) + unitCyl0(0); // x
    unitCyl0_position(1,4) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,4) = youbot_kine.obstacle_position(3,2) + unitCyl0(1)/2; // z

    unitCyl0_position(0,5) = youbot_kine.obstacle_position(3,0) - unitCyl0(0); // x
    unitCyl0_position(1,5) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,5) = youbot_kine.obstacle_position(3,2) + unitCyl0(1)/2; // z

    unitCyl0_position(0,6) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,6) = youbot_kine.obstacle_position(3,1) + unitCyl0(0); // y
    unitCyl0_position(2,6) = youbot_kine.obstacle_position(3,2) + unitCyl0(1)/2; // z

    unitCyl0_position(0,7) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,7) = youbot_kine.obstacle_position(3,1) - unitCyl0(0); // y
    unitCyl0_position(2,7) = youbot_kine.obstacle_position(3,2) + unitCyl0(1)/2; // z

    unitCyl0_position(0,8) = youbot_kine.obstacle_position(3,0) + unitCyl0(0); // x
    unitCyl0_position(1,8) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,8) = youbot_kine.obstacle_position(3,2) - unitCyl0(1)/2; // z

    unitCyl0_position(0,9) = youbot_kine.obstacle_position(3,0) - unitCyl0(0); // x
    unitCyl0_position(1,9) = youbot_kine.obstacle_position(3,1); // y
    unitCyl0_position(2,9) = youbot_kine.obstacle_position(3,2) - unitCyl0(1)/2; // z

    unitCyl0_position(0,10) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,10) = youbot_kine.obstacle_position(3,1) + unitCyl0(0); // y
    unitCyl0_position(2,10) = youbot_kine.obstacle_position(3,2) - unitCyl0(1)/2; // z

    unitCyl0_position(0,11) = youbot_kine.obstacle_position(3,0); // x
    unitCyl0_position(1,11) = youbot_kine.obstacle_position(3,1) - unitCyl0(0); // y
    unitCyl0_position(2,11) = youbot_kine.obstacle_position(3,2) - unitCyl0(1)/2; // z

    // unitCyl1 =======================================================

    unitCyl1_position.resize(3,12);
    unitCyl1_position(0,0) = youbot_kine.obstacle_position(4,0) + unitCyl1(0); // x
    unitCyl1_position(1,0) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,0) = youbot_kine.obstacle_position(4,2); // z

    unitCyl1_position(0,1) = youbot_kine.obstacle_position(4,0) - unitCyl1(0); // x
    unitCyl1_position(1,1) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,1) = youbot_kine.obstacle_position(4,2); // z

    unitCyl1_position(0,2) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,2) = youbot_kine.obstacle_position(4,1) + unitCyl1(0); // y
    unitCyl1_position(2,2) = youbot_kine.obstacle_position(4,2); // z

    unitCyl1_position(0,3) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,3) = youbot_kine.obstacle_position(4,1) - unitCyl1(0); // y
    unitCyl1_position(2,3) = youbot_kine.obstacle_position(4,2); // z

    unitCyl1_position(0,4) = youbot_kine.obstacle_position(4,0) + unitCyl1(0); // x
    unitCyl1_position(1,4) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,4) = youbot_kine.obstacle_position(4,2) + unitCyl1(1)/2; // z

    unitCyl1_position(0,5) = youbot_kine.obstacle_position(4,0) - unitCyl1(0); // x
    unitCyl1_position(1,5) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,5) = youbot_kine.obstacle_position(4,2) + unitCyl1(1)/2; // z

    unitCyl1_position(0,6) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,6) = youbot_kine.obstacle_position(4,1) + unitCyl1(0); // y
    unitCyl1_position(2,6) = youbot_kine.obstacle_position(4,2) + unitCyl1(1)/2; // z

    unitCyl1_position(0,7) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,7) = youbot_kine.obstacle_position(4,1) - unitCyl1(0); // y
    unitCyl1_position(2,7) = youbot_kine.obstacle_position(4,2) + unitCyl1(1)/2; // z

    unitCyl1_position(0,8) = youbot_kine.obstacle_position(4,0) + unitCyl1(0); // x
    unitCyl1_position(1,8) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,8) = youbot_kine.obstacle_position(4,2) - unitCyl1(1)/2; // z

    unitCyl1_position(0,9) = youbot_kine.obstacle_position(4,0) - unitCyl1(0); // x
    unitCyl1_position(1,9) = youbot_kine.obstacle_position(4,1); // y
    unitCyl1_position(2,9) = youbot_kine.obstacle_position(4,2) - unitCyl1(1)/2; // z

    unitCyl1_position(0,10) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,10) = youbot_kine.obstacle_position(4,1) + unitCyl1(0); // y
    unitCyl1_position(2,10) = youbot_kine.obstacle_position(4,2) - unitCyl1(1)/2; // z

    unitCyl1_position(0,11) = youbot_kine.obstacle_position(4,0); // x
    unitCyl1_position(1,11) = youbot_kine.obstacle_position(4,1) - unitCyl1(0); // y
    unitCyl1_position(2,11) = youbot_kine.obstacle_position(4,2) - unitCyl1(1)/2; // z
    /*========================== end of obstacle declaration (dimension) ==================================*/
}

void compute_potentialField(YoubotIkine youbot_kine,YoubotKDL youbot_kdl){
    // scalar parameters
    // if Matrix is not initialised as zero, by default it will create either a large number or very small number in each element
    MatrixXd force_att = MatrixXd::Zero(3,5),force_rep= MatrixXd::Zero(3,5);
    Matrix4d T_init,T_final;
    MatrixXd tau_q = MatrixXd::Zero(5,5);
    MatrixXd tau_q_sum=MatrixXd::Zero(5,1);
    VectorXd current_joint_position=VectorXd::Zero(5);
    MatrixXd desired_joint_position=MatrixXd::Zero(5,3); // 5 joints (row) by number of steps (col)

    // local minima variables
    double noise[2]={0.3,-0.3};
    bool localMin_state =0;
    double difference_q_old_1,difference_q_old_2;
    int count_1=0, count_2=0;

    double zeta, eta; // scalar for att and rep
    double d,rho; // threshold distance for att and rep
    double alpha; //scalar for the torque
    double epsilon; // convergence criteria

    zeta = 0.8;
    eta = 0.5;
    d   = 1.5;
    rho = 0.1;
    alpha = 0.1;
    epsilon = 0.01;

    // compute all the obstacle coordinates
    compute_obstaclePosition(youbot_kine);

    int countPoints=1;//traj_points_jointSpace.rows();
    int updateSize = 0;

    // loop through each message point (9 checkpoint)
    for (int cPoints=0;cPoints<countPoints;cPoints++){
        // gradient descent algorithm
        int cStep;
        for (cStep=0; cStep < 1000000000; cStep++){

            // initialise to zero
            force_att.setZero();
            force_rep.setZero();
            tau_q.setZero();
            // loop through each joint frame
            for (int i=0; i<5 ; i++){

                // find o_init and o_final (last column of transformation matrix
                Vector3d o_init,o_final;

                // only the first step of gradient descent define the o_init and o_final
                if (cStep==0){
                    if (cPoints==0){
                        o_init << 0,0,0;
                        T_final = youbot_kine.forward_kinematics(traj_points_jointSpace.row(cPoints).transpose(),i);
                        o_final = T_final.block(0,3,3,1); // extra last column of T matrix
                    }
                    else{

                        T_init = youbot_kine.forward_kinematics(traj_points_jointSpace.row(cPoints-1).transpose(),i);
                        T_final = youbot_kine.forward_kinematics(traj_points_jointSpace.row(cPoints).transpose(),i);

                        o_init = T_init.block(0,3,3,1);
                        o_final= T_final.block(0,3,3,1);
                    }
                }

                // second step of gradient descent onwards
                T_init = youbot_kine.forward_kinematics(current_joint_position,i);
                o_init = T_init.block(0,3,3,1);

                // compute attractive force==============================
                double difference_o = (o_init-o_final).norm();

                if (difference_o <= d)
                    force_att.block(0,i,3,1)= -zeta*(o_init - o_final);
                else
                    force_att.block(0,i,3,1)= -d*zeta*(o_init - o_final)/difference_o;

                // compute repulsive force================================

                // find the nearest obstacle
                MatrixXd checkDistance_obstacle(4,12);

                for (int k=0; k<12; k++){
                    checkDistance_obstacle(0,k) = (o_init-unitBox1_position.block(0,k,3,1)).norm();
                    checkDistance_obstacle(1,k) = (o_init-unitBox2_position.block(0,k,3,1)).norm();
                    checkDistance_obstacle(2,k) = (o_init-unitCyl0_position.block(0,k,3,1)).norm();
                    checkDistance_obstacle(3,k) = (o_init-unitCyl1_position.block(0,k,3,1)).norm();
                }
                // look for the smallest distance
                MatrixXf::Index minRow,minCol;
                double shortestDistance_obstacle = checkDistance_obstacle.minCoeff(&minRow,&minCol);

//                std::cout<<shortestDistance_obstacle<<" "<<minRow<<" "<<minCol<<"\n"<<std::endl;
//                std::cout<<"\n"<<std::endl;

                if (shortestDistance_obstacle<=rho){
                    if (minRow==0){ //box 1
                        force_rep.block(0,i,3,1)= eta*(1/shortestDistance_obstacle-1/rho)* \
                            1/pow(shortestDistance_obstacle,2) * (o_init-unitBox1_position.block(0,minCol,3,1))/shortestDistance_obstacle;
                    }
                    else if (minRow==1){ //box 2
                        force_rep.block(0,i,3,1)= eta*(1/shortestDistance_obstacle-1/rho)* \
                            1/pow(shortestDistance_obstacle,2) * (o_init-unitBox2_position.block(0,minCol,3,1))/shortestDistance_obstacle;
                    }
                    else if (minRow==2){ //cyl 0
                        force_rep.block(0,i,3,1)= eta*(1/shortestDistance_obstacle-1/rho)* \
                            1/pow(shortestDistance_obstacle,2) * (o_init-unitCyl0_position.block(0,minCol,3,1))/shortestDistance_obstacle;
                    } else{ //cyl 1
                        force_rep.block(0,i,3,1)= eta*(1/shortestDistance_obstacle-1/rho)* \
                            1/pow(shortestDistance_obstacle,2) * (o_init-unitCyl1_position.block(0,minCol,3,1))/shortestDistance_obstacle;
                    }

                } else
                    force_rep.block(0,i,3,1)<< 0,0,0;

                // find jacobian ==================================
                MatrixXd jacobian_o=MatrixXd::Zero(3,5);
                jacobian_o = youbot_kine.get_jacobian(current_joint_position,i).block(0,0,3,5);

                tau_q.block(0,i,5,1) = jacobian_o.transpose()*force_att.block(0,i,3,1) + \
                                         jacobian_o.transpose()*force_rep.block(0,i,3,1);


//                std::cout<<"\nDEBUGGING!!!  "<<i<<"\n"<<std::endl;
//                std::cout<<"Jacobian_o \n"<<jacobian_o<<"\n"<<std::endl;
//                std::cout<<"Attractive: \n"<<force_att<<"\n"<<std::endl;
//                std::cout<<"Repulsive:  \n"<<force_rep<<"\n"<<std::endl;
//                std::cout<<"Torque: \n"<<tau_q<<"\n"<<std::endl;
//                std::cout<<"Init_o: \n"<<o_init<<"\n"<<std::endl;
//                std::cout<<"Final_o:\n"<<o_final<<"\n"<<std::endl;
//                std::cout<<"\n end debug\n"<<std::endl;
            } // end for i loop (each joint)

            // sum all of the tau_q to create 5x1 vector
            tau_q_sum = tau_q.rowwise().sum();
//            std::cout<<tau_q<<"\n sum of tau q\n"<<tau_q_sum<<std::endl;
//            std::cout<<" "<<std::endl;


            // main gradient descent algorithm
            if (cStep>2)
                desired_joint_position.conservativeResize(5,cStep+1);

            desired_joint_position.block(0,cStep,5,1) = current_joint_position + alpha*tau_q_sum/tau_q_sum.norm();

            // check convergence
            VectorXd final_q(5),current_q(5);
            current_q = desired_joint_position.block(0,cStep,5,1);
            final_q = traj_points_jointSpace.block(cPoints,0,1,5).transpose();

            double difference_in_q = (current_q - final_q).norm();

            std::cout<<"Init_q: \n"<<current_q<<"\n"<<std::endl;
            std::cout<<"Final_q:\n"<<final_q<<"\n"<<std::endl;
            std::cout<<"\nDifference: "<<difference_in_q<<" ... "<<cStep<<std::endl;

            if ( difference_in_q < epsilon ){
                std::cout<<current_q<<"    "<<final_q<<std::endl;
                break;
            }

            // check if stuck in local minima

//            if (cStep%2==0){ //even step
//                if (difference_q_old_1==difference_in_q){
//                    count_1 = count_1 + 1;
//                }
//                if (count_1>20){
//                    localMin_state=1;
//                    count_1 = 0;
//                }
//                else{
//                    localMin_state = 0; // reset the state
//                }
//                difference_q_old_1 = difference_in_q;
//            }
//            else{
//                if (difference_q_old_2==difference_in_q){
//                    count_2 = count_2 +1;
//                }
//                if (count_2>20){
//                    localMin_state=1;
//                    count_2 = 0;
//                }
//                else{
//                    localMin_state = 0; // reset the state
//                }
//                difference_q_old_2 = difference_in_q;
//            }
            std::cout<<"\n"<<count_1<<"....."<<count_2<<std::endl;
            if (localMin_state){
                VectorXd noiseVec(5);
                noiseVec << noise[rand()%2],noise[rand()%2],noise[rand()%2],noise[rand()%2],noise[rand()%2];
                current_joint_position = current_joint_position + noiseVec;
                std::cout<<"Noise = "<<noiseVec<<std::endl;
            }
            else{
                current_joint_position = desired_joint_position.block(0,cStep,5,1);
            }







        }// end for cStep loop (gradient descent algorithm)
        std::cout<<"cStep: "<<cStep<<std::endl;
        std::cout<<"noCols: "<<desired_joint_position.cols()<<std::endl;
//        std::cout<<desired_joint_position<<"\n"<<std::endl;
//        std::cout<<traj_points_jointSpace.block(cPoints,0,1,5).transpose()<<std::endl;
        // store all the previous joints configuration that avoid obstacle
        double noCol = desired_joint_position.cols();

        traj_pts_jointSpace_avoidObstacle.conservativeResize(5,noCol);

        traj_pts_jointSpace_avoidObstacle.block(0,cPoints,5,noCol) = desired_joint_position;


    }// end for cPoint loop

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4c");


    YoubotIkine youbot_kine;
    YoubotKDL youbot_kdl;
    youbot_kine.init();
    youbot_kdl.init();

    MatrixXd check_point_matrix = get_checkpoint();

    traj_q4c(check_point_matrix,youbot_kine,youbot_kdl);

    int countPoints=traj_points_jointSpace.rows();



    std::cout<<countPoints<<std::endl;

    // spinOnce cannot get the obstacle position
    for (int i = 0; i<3;i++){
        ros::spinOnce();
        ros::Duration(0.3).sleep();
        std::cout<<i<<"...Obstacle Position\n"<<youbot_kine.obstacle_position<<std::endl;
    }

    compute_potentialField(youbot_kine,youbot_kdl);

    while (ros::ok())
    {
        int countPoints = traj_pts_jointSpace_avoidObstacle.cols();



        for (int cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout << "start " << cPoints << "\n" << std::endl;

            VectorXd desiredJointPosition = traj_pts_jointSpace_avoidObstacle.block(0,cPoints,5,1);
            std::cout<<desiredJointPosition<<std::endl;


            traj_pt.positions.clear();
            for (int i = 0; i < 5; i++) {
                traj_pt.positions.push_back(desiredJointPosition(i));
            }
            double dt = 1;//traj_points(cPoints, 6);
            youbot_kine.publish_trajectory(traj_pt, dt * 1e9);

            ros::Duration(dt).sleep();
            std::cout << "after publish\n" << std::endl;

            ros::spinOnce();
        }
    }

    return 0;
}
