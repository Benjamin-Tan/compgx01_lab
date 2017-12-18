#include <ros/ros.h>
#include <cw2_helper/YoubotIkine.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <unsupported/Eigen/MatrixFunctions>
#include <inverse_kinematics/YoubotKDL.h>

trajectory_msgs::JointTrajectoryPoint traj_pt;
MatrixXd traj_points;
int obstacleNo = 0; // label obstacle, 1: box1, 2: box2, 3: cyl0, 4: cyl1

struct obstacle{
    // 0:x, 1:y, 2:z
    float centre[3];
    float length[3];
};
obstacle box1,box2,cyl0,cyl1;

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;

    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,3);

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_q4d.bag", rosbag::bagmode::Read);
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

void assignObstacle_properties(YoubotIkine youbot_kine){
    // assign obstacle position x,y,z into each class
    for (int i = 0; i<3; i++){
        box1.centre[i] = youbot_kine.obstacle_position(1,i);
        box2.centre[i] = youbot_kine.obstacle_position(2,i);
        cyl0.centre[i] = youbot_kine.obstacle_position(3,i);
        cyl1.centre[i] = youbot_kine.obstacle_position(4,i);
    }

    // assign length properties into each obstacle
    box1.length[0] = 0.194033;
    box1.length[1] = 0.077038;
    box1.length[2] = 0.19810;

    box2.length[0] = 0.167065;
    box2.length[1] = 0.112100;
    box2.length[2] = 0.104652;

    cyl0.length[0] = 0.05; //radius
    cyl0.length[1] = 0.08; //length
    cyl0.length[2] = 0;

    cyl1.length[0] = 0.06; //radius
    cyl1.length[1] = 0.14; //length
    cyl1.length[2] = 0;

}

bool checkCollision_node_cart(VectorXd joints_cart, YoubotIkine youbot_kine){
    // return true if collide
    Vector3d pose_random = joints_cart;
    bool collideState = 0;
    // check every obstacle
    // box1
    if (abs(pose_random(0)-box1.centre[0])<= box1.length[0]/2 && \
        abs(pose_random(1)-box1.centre[1])<= box1.length[1]/2 && \
        abs(pose_random(2)-box1.centre[2])<= box1.length[2]/2){

        collideState = 1;
        obstacleNo = 1;
    }
    else{
        collideState = 0;
        obstacleNo = 0;
    }

    if (abs(pose_random(0)-box2.centre[0])<= box2.length[0]/2 && \
        abs(pose_random(1)-box2.centre[1])<= box2.length[1]/2 && \
        abs(pose_random(2)-box2.centre[2])<= box2.length[2]/2){

        collideState = 1;
        obstacleNo = 2;
    }
    else{
        collideState = 0;
        obstacleNo = 0;
    }

    // cylinder
    // (x-x_o)^2 + (y-y_o)^2 <= r^2 + (z-z_o)^2
    if ((pow(pose_random(0)-cyl0.centre[0],2) + pow(pose_random(1)-cyl0.centre[1],2)) \
            <= (pow(cyl0.length[0],2) + pow(pose_random(2)-cyl0.centre[2],2)) && \
        abs(pose_random(2)-cyl0.centre[2])<= cyl0.length[1]/2){

        collideState = 1;
        obstacleNo = 3;
    }
    else{
        collideState = 0;
        obstacleNo = 0;
    }

    if ((pow(pose_random(0)-cyl1.centre[0],2) + pow(pose_random(1)-cyl1.centre[1],2)) \
            <= (pow(cyl1.length[0],2) + pow(pose_random(2)-cyl1.centre[2],2)) && \
        abs(pose_random(2)-cyl1.centre[2])<= cyl1.length[1]/2){

        collideState = 1;
        obstacleNo = 4;
    }
    else{
        collideState = 0;
        obstacleNo = 0;
    }

    return collideState;
}

void traj_q4d (MatrixXd checkpoint, YoubotIkine youbot_kine, YoubotKDL youbot_kdl)
{
    // Do something
    // Converts the checkpoint matrix into trajectory point messages

    double dt = 0.01;

    int countMessage = checkpoint.rows();
    int updateSize = 0;
    int collideState = 0;

    Vector3d sol_P, cur_P, final_P;
    traj_points.resize(5,3); // 1st 3 columns are translation(3)

    for (int cMessage = 0; cMessage<countMessage; cMessage++){

        int totalStep = 1/dt;

        traj_points.conservativeResize(updateSize + totalStep , 3);

        // assign checkpoint into current T and final T between 2 pose
        if (cMessage==0){
            cur_P << youbot_kine.forward_kinematics(youbot_kine.current_joint_position,4).block(0,3,3,1);
            final_P=checkpoint.row(cMessage).transpose();
        }
        else{
            cur_P = checkpoint.row(cMessage-1).transpose();
            final_P=checkpoint.row(cMessage).transpose();
        }

        std::cout<<"CurP: "<<cur_P.transpose()<<std::endl;
        std::cout<<"FinP: "<<final_P.transpose()<<std::endl;

        int cStep=0;
        float increaseMagnitude = 1.2;

        for (cStep=0; cStep<totalStep; cStep++){

            sol_P = cur_P + dt*cStep*(final_P - cur_P);

            Vector3d obstacle_centre = Vector3d::Zero();

            if (checkCollision_node_cart(sol_P,youbot_kine)){
                collideState = 1;

                // determine which obstacle it collides
                if (obstacleNo==1){
                    // collide with box1
                    obstacle_centre << box1.centre[0],box1.centre[1],box1.centre[2];
                }else if (obstacleNo==2){
                    // collide with box2
                    obstacle_centre << box2.centre[0],box2.centre[1],box2.centre[2];
                }else if (obstacleNo==3){
                    // collide with cyl0
                    obstacle_centre << cyl0.centre[0],cyl0.centre[1],cyl0.centre[2];
                }else{
                    // collide with cyl1
                    obstacle_centre << cyl1.centre[0],cyl1.centre[1],cyl1.centre[2];
                }
                // push the position out of the obstacle at a direction relative to its centre point (vector operation)
                sol_P = obstacle_centre + increaseMagnitude*(sol_P-obstacle_centre);
            }else {
                collideState = 0;
            }
//            std::cout<<"SolP: "<<sol_P.transpose()<<std::endl;
//            std::cout<<"Collide: "<<collideState<<std::endl;
//            std::cout<<"\n"<<std::endl;

            traj_points.block(cStep + updateSize,0,1,3) = sol_P.transpose();
        }
        // compute last time step
        if (cMessage == 9){
            traj_points.conservativeResize(updateSize + totalStep + 1 , 3);

            sol_P = cur_P + dt*cStep*(final_P - cur_P);

            traj_points.block(totalStep + updateSize,0,1,3) = sol_P.transpose();

//            std::cout<<"solution:\n"<<sol_P<<"\n"<<std::endl;
//            std::cout<<"trajpts: \n"<<traj_points<<"\n"<<std::endl;
//            std::cout<<"  "<<std::endl;
        }
        updateSize = traj_points.rows();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4d");

    YoubotIkine youbot_kine;
    YoubotKDL youbot_kdl;
    youbot_kine.init();
    youbot_kdl.init();

    MatrixXd check_point_matrix = get_checkpoint();

    // spinOnce cannot get the obstacle position
    for (int i = 0; i<3;i++){
        ros::spinOnce();
        ros::Duration(0.3).sleep();
        std::cout<<i<<"...Obstacle Position\n"<<youbot_kine.obstacle_position<<std::endl;
    }

    // compute all the obstacle coordinates
    assignObstacle_properties(youbot_kine);

    traj_q4d(check_point_matrix,youbot_kine,youbot_kdl);
    int countPoints=traj_points.rows();
    int cPoints = 0;
    std::cout<<countPoints<<std::endl;
    std::cout<<"New\n"<<traj_points<<std::endl;

    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout<<"start "<< cPoints <<"\n"<<std::endl;

            // KDL
            Matrix4d desiredPose;
            desiredPose <<  1,0,0,traj_points(cPoints,0),
                            0,1,0,traj_points(cPoints,1),
                            0,0,1,traj_points(cPoints,2),
                            0,0,0,1;

            KDL::Frame desiredPose_KDL = youbot_kine.poseMatrix_kdlFrame(desiredPose);
            KDL::JntArray desiredJointPosition = youbot_kdl.inverse_kinematics_closed(desiredPose_KDL);

            std::cout<<desiredJointPosition.data.transpose()<<std::endl;

            traj_pt.positions.clear();
            for (int i=0; i<5; i++){
                traj_pt.positions.push_back(desiredJointPosition.data(i));
            }
            double dt = 0.1;
            youbot_kine.publish_trajectory(traj_pt,dt*1e9);

            ros::Duration(dt).sleep();
            std::cout<<"after publish\n"<<std::endl;

            ros::spinOnce();
        }
    }

    return 0;
}
