#ifndef CW2_HELPER_YOUBOTMANUAL_H
#define CW2_HELPER_YOUBOTMANUAL_H

#include <inverse_kinematics/YoubotKinematics.h>
#include <gazebo_msgs/LinkStates.h>


using namespace Eigen;

class YoubotIkine : public YoubotKinematics
{

private:
    Matrix4d current_pose, desired_pose;
    VectorXd desired_joint_position;
    MatrixXd jacobian;

    ros::Subscriber subscriber_obstacle;
    ros::Publisher publisher;

public:

    VectorXd current_joint_position;
    MatrixXd obstacle_position;

    void init();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    MatrixXd get_jacobian(Eigen::VectorXd current_pose,int k);
    VectorXd inverse_kinematics_closed(Eigen::Matrix4d desired_pose);
    VectorXd inverse_kinematics_jac(Eigen::VectorXd desired_pose_vec);
    Matrix4d forward_kinematics(Eigen::VectorXd current_joint_position, int count);
    bool check_singularity(VectorXd joint_position);

    KDL::Frame poseMatrix_kdlFrame(Matrix4d pose);
    VectorXd rotationMatrix_Vector(Matrix4d rotationMatrix);
    Matrix4d rotationVector_Matrix(VectorXd rotationVector);
    VectorXd pose_rotationVec(geometry_msgs::TransformStamped pose);
    Matrix4d pose_rotationMat(geometry_msgs::TransformStamped pose);
    void publish_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory,int dt);
    Matrix4d vectorQuat_rotationMat(VectorXd pose);

    void obstacle_callback(const gazebo_msgs::LinkStates::ConstPtr &w);
};

#endif //CW2_HELPER_YOUBOTMANUAL_H

