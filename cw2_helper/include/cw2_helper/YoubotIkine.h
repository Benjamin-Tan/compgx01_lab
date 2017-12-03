#ifndef CW2_HELPER_YOUBOTMANUAL_H
#define CW2_HELPER_YOUBOTMANUAL_H

#include <inverse_kinematics/YoubotKinematics.h>

using namespace Eigen;

class YoubotIkine : public YoubotKinematics
{

private:
    Matrix4d current_pose, desired_pose;
    VectorXd desired_joint_position;
    MatrixXd jacobian;

public:
    VectorXd current_joint_position;

    void init();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    MatrixXd get_jacobian(Eigen::VectorXd current_pose);
    VectorXd inverse_kinematics_closed(Eigen::Matrix4d desired_pose);
    VectorXd inverse_kinematics_jac(Eigen::Matrix4d desired_pose);
    Matrix4d forward_kinematics(Eigen::VectorXd current_joint_position, int count);
    bool check_singularity(VectorXd joint_position);

    VectorXd rotationMatrix_Vector(Matrix4d rotationMatrix);
    VectorXd pose_rotationVec(geometry_msgs::TransformStamped pose);
    Matrix4d pose_rotationMat(geometry_msgs::TransformStamped pose);
};

#endif //CW2_HELPER_YOUBOTMANUAL_H

