#include <cw2_helper/YoubotIkine.h>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <gazebo_msgs/LinkStates.h>

void YoubotIkine::init()
{
    YoubotKinematics::init();
    subscriber_joint_state = n.subscribe<sensor_msgs::JointState>("/joint_states", 5, &YoubotIkine::joint_state_callback,
                                                                  this);
    subscriber_obstacle = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",5,&YoubotIkine::obstacle_callback,this);

    current_joint_position.resize(5);
    desired_joint_position.resize(5);
    jacobian.resize(6, 5);

    this->publisher = this->n.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 3);
}

void YoubotIkine::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q) {
    for (int i = 0; i < 5; i++)
        current_joint_position(i) = DH_params[i][3] - q->position.at(i);
}

void YoubotIkine::obstacle_callback(const gazebo_msgs::LinkStates::ConstPtr &w){
    obstacle_position.resize(5,3);

    for (int i = 0; i < 5; i++)
    {
        obstacle_position(i,0) =w->pose[i+1].position.x;
        obstacle_position(i,1) =w->pose[i+1].position.y;
        obstacle_position(i,2) =w->pose[i+1].position.z;
    }
}


MatrixXd YoubotIkine::get_jacobian(VectorXd current_pose,int k)
{
    //Add jacobian code. (without using KDL libraries)
    Matrix4d forwardTransform_1 = forward_kinematics(current_pose,0);
    Matrix4d forwardTransform_2 = forward_kinematics(current_pose,1);
    Matrix4d forwardTransform_3 = forward_kinematics(current_pose,2);
    Matrix4d forwardTransform_4 = forward_kinematics(current_pose,3);
    Matrix4d forwardTransform_5 = forward_kinematics(current_pose,4);

    Vector3d z0(0,0,1),z1,z2,z3,z4;
    Vector3d o0(0,0,0),o1,o2,o3,o4,o5;
    Vector3d jacV1,jacV2,jacV3,jacV4,jacV5, jacW1,jacW2,jacW3,jacW4,jacW5;

    // Compute the vector z and o
    for (int i = 0; i<3 ; i++){
        z1(i) = forwardTransform_1(i,2);
        z2(i) = forwardTransform_2(i,2);
        z3(i) = forwardTransform_3(i,2);
        z4(i) = forwardTransform_4(i,2);

        o1(i) = forwardTransform_1(i,3);
        o2(i) = forwardTransform_2(i,3);
        o3(i) = forwardTransform_3(i,3);
        o4(i) = forwardTransform_4(i,3);
        o5(i) = forwardTransform_5(i,3);
    }

    // Compute Jacobian Matrix
    if (k==0){
        jacV1 = z0.cross(o1-o0);
        jacV2 << 0,0,0;
        jacV3 << 0,0,0;
        jacV4 << 0,0,0;
        jacV5 << 0,0,0;
    }
    else if (k==1){
        jacV1 = z0.cross(o2-o0);
        jacV2 = z1.cross(o2-o1);
        jacV3 << 0,0,0;
        jacV4 << 0,0,0;
        jacV5 << 0,0,0;
    }
    else if (k==2){
        jacV1 = z0.cross(o3-o0);
        jacV2 = z1.cross(o3-o1);
        jacV3 = z2.cross(o3-o2);
        jacV4 << 0,0,0;
        jacV5 << 0,0,0;
    }
    else if (k==3){
        jacV1 = z0.cross(o4-o0);
        jacV2 = z1.cross(o4-o1);
        jacV3 = z2.cross(o4-o2);
        jacV4 = z3.cross(o4-o3);
        jacV5 << 0,0,0;
    }
    else{
        jacV1 = z0.cross(o5-o0);
        jacV2 = z1.cross(o5-o1);
        jacV3 = z2.cross(o5-o2);
        jacV4 = z3.cross(o5-o3);
        jacV5 = z4.cross(o5-o4);
    }

    jacW1 = z0; jacW2 = z1; jacW3 = z2; jacW4 = z3; jacW5 = z4;

    // Assigning the individual Jacobian elements into the matrix
    jacobian.topRows(3) << jacV1,jacV2,jacV3,jacV4,jacV5;
    jacobian.bottomRows(3) << jacW1,jacW2,jacW3,jacW4,jacW5;

    return jacobian;
}

VectorXd YoubotIkine::inverse_kinematics_closed(Matrix4d desired_pose)
{
    //Add closed-form inverse kinematics code. (without using KDL libraries)
    double r11,r12,r13, r21,r22,r23, r31,r32,r33, x,y,z;
    VectorXd ik_theta(5), ik_theta_1(5),ik_theta_2(5), ik_theta_3(5), ik_theta_real(5);
    double G,A,B,C,D;

    r11 = desired_pose(0,0); r12 = desired_pose(0,1); r13 = desired_pose(0,2); x = desired_pose(0,3);
    r21 = desired_pose(1,0); r22 = desired_pose(1,1); r23 = desired_pose(1,2); y = desired_pose(1,3);
    r31 = desired_pose(2,0); r32 = desired_pose(2,1); r33 = desired_pose(2,2); z = desired_pose(2,3);

    ik_theta(0) = atan(y/x) + M_PI;
    ik_theta_1(0) = ik_theta(0);
    ik_theta_2(0) = atan(y/x) - M_PI;
    ik_theta_3(0) = ik_theta_2(0);

//    std::cout << "Before "<<ik_theta(0) << std::endl;
//    ik_theta(0) = DH_params[0][3] - ik_theta(0);
//    std::cout << "After  "<<ik_theta(0) << std::endl;
//
//
//    // only 1 theta is not correct.
//    if ((ik_theta(0)>(169*M_PI/180) && ik_theta(0)<(M_PI+169*M_PI/180)) || (ik_theta(0)<(169*M_PI/180) && ik_theta(0)>(M_PI-169*M_PI/180))) {
//        ik_theta(0) = ik_theta(0) - M_PI;
//        std::cout << 999 << std::endl;
//    }
//    else if ((ik_theta(0)<(-169*M_PI/180) && ik_theta(0)>(-M_PI-169*M_PI/180)) || (ik_theta(0)>(-169*M_PI/180) && ik_theta(0)<(-M_PI+169*M_PI/180))){
//        ik_theta(0)=ik_theta(0)+M_PI;
//        std::cout<<666<<std::endl;
//    }
//    else
//        ik_theta(0)=ik_theta(0);

//    std::cout<<r32<<" "<<r31<<" "<<r22<<" "<<r12<<" "<< std::endl;
// why is this not working? theoretical value
    double y5 = r21*cos(ik_theta(0)) - r11*sin(ik_theta(0));
    double x5 = r22*cos(ik_theta(0)) - r12*sin(ik_theta(0));
    ik_theta(4) = atan2(y5,x5);
    ik_theta_1(4) = ik_theta(4);

    y5 = r21*cos(ik_theta_2(0)) - r11*sin(ik_theta_2(0));
    x5 = r22*cos(ik_theta_2(0)) - r12*sin(ik_theta_2(0));
    ik_theta_2(4) = atan2(y5,x5);
    ik_theta_3(4) = ik_theta_2(4);
//    std::cout << "Before "<<ik_theta(4) << std::endl;
//    ik_theta(4) = DH_params[4][3] - ik_theta(4);
//    std::cout << "After  "<<ik_theta(4) << std::endl;
    // somehow correct value
//    ik_theta(4) = DH_params[4][3] -atan2(-r32,r31) - M_PI;


    G = pow( -1000*x/cos(ik_theta(0)) - 183*r31/cos(ik_theta(4)) + 33 ,2) + pow(1000*z - 147 - 183*r33,2);
    ik_theta(2) = acos( (G-pow(135,2)-pow(155,2)) / (2*135*155) );

    G = pow( -1000*x/cos(ik_theta_1(0)) - 183*r31/cos(ik_theta_1(4)) + 33 ,2) + pow(1000*z - 147 - 183*r33,2);
    ik_theta_1(2) = -acos( (G-pow(135,2)-pow(155,2)) / (2*135*155) );

    G = pow( -1000*x/cos(ik_theta_2(0)) - 183*r31/cos(ik_theta_2(4)) + 33 ,2) + pow(1000*z - 147 - 183*r33,2);
    ik_theta_2(2) = acos( (G-pow(135,2)-pow(155,2)) / (2*135*155) );

    G = pow( -1000*x/cos(ik_theta_3(0)) - 183*r31/cos(ik_theta_3(4)) + 33 ,2) + pow(1000*z - 147 - 183*r33,2);
    ik_theta_3(2) = -acos( (G-pow(135,2)-pow(155,2)) / (2*135*155) );

    // Check isnan, if it is, max its position
    if (std::isnan(ik_theta(2)))
        ik_theta(2) = -(148)*M_PI/180;

    if (std::isnan(ik_theta_1(2)))
        ik_theta_1(2) = -(148)*M_PI/180;

    if (std::isnan(ik_theta_2(2)))
        ik_theta_2(2) = -(148)*M_PI/180;

    if (std::isnan(ik_theta_3(2)))
        ik_theta_3(2) = -(148)*M_PI/180;


    A = 135*cos(ik_theta(2)) + 155;
    B = 135*sin(ik_theta(2));
    C = -1000*x/cos(ik_theta(0)) + 33 - 183*r31/cos(ik_theta(4));
    D = 1000*z - 147 - 183*r33;
    ik_theta(1) = atan2(C,D) - atan2(B,A);
    ik_theta(3) = atan2(r31,r33*cos(ik_theta(4))) - ik_theta(1) - ik_theta(2);

    A = 135*cos(ik_theta_1(2)) + 155;
    B = 135*sin(ik_theta_1(2));
    C = -1000*x/cos(ik_theta_1(0)) + 33 - 183*r31/cos(ik_theta_1(4));
    D = 1000*z - 147 - 183*r33;
    ik_theta_1(1) = atan2(C,D) - atan2(B,A);
    ik_theta_1(3) = atan2(r31,r33*cos(ik_theta_1(4))) - ik_theta_1(1) - ik_theta_1(2);

    A = 135*cos(ik_theta_2(2)) + 155;
    B = 135*sin(ik_theta_2(2));
    C = -1000*x/cos(ik_theta_2(0)) + 33 - 183*r31/cos(ik_theta_2(4));
    D = 1000*z - 147 - 183*r33;
    ik_theta_2(1) = atan2(C,D) - atan2(B,A);
    ik_theta_2(3) = atan2(r31,r33*cos(ik_theta_2(4))) - ik_theta_2(1) - ik_theta_2(2);

    A = 135*cos(ik_theta_3(2)) + 155;
    B = 135*sin(ik_theta_3(2));
    C = -1000*x/cos(ik_theta_3(0)) + 33 - 183*r31/cos(ik_theta_3(4));
    D = 1000*z - 147 - 183*r33;
    ik_theta_3(1) = atan2(C,D) - atan2(B,A);
    ik_theta_3(3) = atan2(r31,r33*cos(ik_theta_3(4))) - ik_theta_3(1) - ik_theta_3(2);



//    for (int i = 1; i < 5; i++){
//        ik_theta(i) = DH_params[i][3] - ik_theta(i);
//        ik_theta_1(i) = DH_params[i][3] - ik_theta_1(i);
//        ik_theta_2(i) = DH_params[i][3] - ik_theta_2(i);
//        ik_theta_3(i) = DH_params[i][3] - ik_theta_3(i);
//    }


    // Check the joint limits of 1 and 5
    if (ik_theta(0) > 0 && ik_theta(0) < 338*M_PI/180) {
        ik_theta_real(0) = ik_theta(0);
        ik_theta_real(4) = ik_theta(4);
    }
    else if (ik_theta_2(0) > 0 && ik_theta_2(0) < 338*M_PI/180) {
        ik_theta_real(0) = ik_theta_2(0);
        ik_theta_real(4) = ik_theta_2(4);
    }

    // Check the joint limits of 3,2 and 4
    if (ik_theta(2) > -297*M_PI/180 && ik_theta(2) < 0 && ik_theta(1) < 155*M_PI/180){
        ik_theta_real(2) = ik_theta(2);
        ik_theta_real(1) = ik_theta(1);
        ik_theta_real(3) = ik_theta(3);
    }
    else if (ik_theta_1(2) > -297*M_PI/180 && ik_theta_1(2) < 0 && ik_theta_1(1) < 155*M_PI/180){
        ik_theta_real(2) = ik_theta_1(2);
        ik_theta_real(1) = ik_theta_1(1);
        ik_theta_real(3) = ik_theta_1(3);
    }

    // Check if joint 5 exceeds 2*pi
    if (ik_theta_real(4) > 2*M_PI)
        ik_theta_real(4) = ik_theta_real(4) - 2*M_PI;







    std::cout<< "\n"<< ik_theta.transpose()<<std::endl;
    std::cout << ik_theta_1.transpose()<<std::endl;
    std::cout<< ik_theta_2.transpose()<<std::endl;
    std::cout<< ik_theta_3.transpose()<<std::endl;
//
//    std::cout<<"\nFinal"<<std::endl;
//    std::cout<<ik_theta_real(0)<<" "<<ik_theta_real(1)<<" "<<ik_theta_real(2)<<" "<<ik_theta_real(3)<<" "<<ik_theta_real(4)<<std::endl;

    return ik_theta_real;

}

VectorXd YoubotIkine::inverse_kinematics_jac(VectorXd desired_pose_vec)
{
    //Add iterative inverse kinematics code. (without using KDL libraries)
    VectorXd previous_joint_position(5);
    // reverse the offset[-pi,pi] to raw [0,2pi];
    for (int i = 1; i < 5; i++){
        previous_joint_position(i) = DH_params[i][3] - current_joint_position(i);
    }

    float lambda=0.2;

    for (int k=0;k<100000;k++){
        Matrix4d previous_pose = forward_kinematics(previous_joint_position,4);
        VectorXd previous_pose_vec = rotationMatrix_Vector(previous_pose);

        MatrixXd jacobMat = get_jacobian(previous_joint_position,5);

        desired_joint_position = previous_joint_position + lambda*jacobMat.transpose()*(desired_pose_vec - previous_pose_vec);

        // re-adjust the limits
        for (int i=0;i<5; i++){
            while (desired_joint_position(i)<-M_PI)
                desired_joint_position(i) = desired_joint_position(i) + 2*M_PI;
            while (desired_joint_position(i)>=M_PI)
                desired_joint_position(i) = desired_joint_position(i) - 2*M_PI;
        }


        if ((desired_pose_vec-previous_pose_vec).norm() < 0.001 || check_singularity(desired_joint_position)) {
            desired_joint_position = previous_joint_position;
//            for (int i = 0; i < 5; i++)
//                desired_joint_position(i) = DH_params[i][3] - desired_joint_position(i);
            break;
        }

        previous_joint_position = desired_joint_position;


    }

//    for (int i = 0; i < 5; i++)
//        desired_joint_position(i) = DH_params[i][3] - desired_joint_position(i);
    return desired_joint_position;
}

Matrix4d YoubotIkine::forward_kinematics(VectorXd current_joint_position,int count) {

    //Add forward kinematics code. (without using KDL libraries)
    Matrix4d T_01, T_12, T_23, T_34, T_45, T_05;

    VectorXd theta(5);
    for (int i = 0; i < 5; i++)
//            theta(i) = current_joint_position(i);
        theta(i) = DH_params[i][3]-current_joint_position(i);

    T_01 << cos(theta(0)), -sin(theta(0)) * cos(DH_params[0][1]),  sin(theta(0)) * sin(DH_params[0][1]), DH_params[0][0] * cos(theta(0)),
            sin(theta(0)),  cos(theta(0)) * cos(DH_params[0][1]), -cos(theta(0)) * sin(DH_params[0][1]), DH_params[0][0] * sin(theta(0)),
                        0,                  sin(DH_params[0][1]),                  cos(DH_params[0][1]),                 DH_params[0][2],
                        0,                                     0,                                     0,                               1;
    T_12 << cos(theta(1)), -sin(theta(1)) * cos(DH_params[1][1]),  sin(theta(1)) * sin(DH_params[1][1]), DH_params[1][0] * cos(theta(1)),
            sin(theta(1)),  cos(theta(1)) * cos(DH_params[1][1]), -cos(theta(1)) * sin(DH_params[1][1]), DH_params[1][0] * sin(theta(1)),
                        0,                  sin(DH_params[1][1]),                  cos(DH_params[1][1]),                 DH_params[1][2],
                        0,                                     0,                                     0,                               1;
    T_23 << cos(theta(2)), -sin(theta(2)) * cos(DH_params[2][1]),  sin(theta(2)) * sin(DH_params[2][1]), DH_params[2][0] * cos(theta(2)),
            sin(theta(2)),  cos(theta(2)) * cos(DH_params[2][1]), -cos(theta(2)) * sin(DH_params[2][1]), DH_params[2][0] * sin(theta(2)),
                        0,                  sin(DH_params[2][1]),                  cos(DH_params[2][1]),                 DH_params[2][2],
                        0,                                     0,                                     0,                               1;
    T_34 << cos(theta(3)), -sin(theta(3)) * cos(DH_params[3][1]),  sin(theta(3)) * sin(DH_params[3][1]), DH_params[3][0] * cos(theta(3)),
            sin(theta(3)),  cos(theta(3)) * cos(DH_params[3][1]), -cos(theta(3)) * sin(DH_params[3][1]), DH_params[3][0] * sin(theta(3)),
                        0,                  sin(DH_params[3][1]),                  cos(DH_params[3][1]),                 DH_params[3][2],
                        0,                                     0,                                     0,                               1;
    T_45 << cos(theta(4)), -sin(theta(4)) * cos(DH_params[4][1]),  sin(theta(4)) * sin(DH_params[4][1]), DH_params[4][0] * cos(theta(4)),
            sin(theta(4)),  cos(theta(4)) * cos(DH_params[4][1]), -cos(theta(4)) * sin(DH_params[4][1]), DH_params[4][0] * sin(theta(4)),
                        0,                  sin(DH_params[4][1]),                  cos(DH_params[4][1]),                 DH_params[4][2],
                        0,                                     0,                                     0,                               1;

    if (count==0)
        T_05 = T_01;
    else if (count==1)
        T_05 = T_01 * T_12;
    else if (count==2)
        T_05 = T_01 * T_12 * T_23;
    else if (count==3)
        T_05 = T_01 * T_12 * T_23 * T_34;
    else
        T_05 = T_01 * T_12 * T_23 * T_34 * T_45;


    return T_05;
}


bool YoubotIkine::check_singularity(VectorXd joint_position)
{
    //Add singularity checker. (without using KDL libraries)
    if ((jacobian.transpose()*jacobian).determinant()<0.0001)
        return 1;
    else
        return 0;
}

VectorXd YoubotIkine::rotationMatrix_Vector(Matrix4d rotationMatrix){
    VectorXd rotationVector(6);

    rotationVector(0) = rotationMatrix(0,3);
    rotationVector(1) = rotationMatrix(1,3);
    rotationVector(2) = rotationMatrix(2,3);

    // Convert Rotation Matrix to Axis-angle
    float r_11 = rotationMatrix(0,0);
    float r_12 = rotationMatrix(0,1);
    float r_13 = rotationMatrix(0,2);
    float r_21 = rotationMatrix(1,0);
    float r_22 = rotationMatrix(1,1);
    float r_23 = rotationMatrix(1,2);
    float r_31 = rotationMatrix(2,0);
    float r_32 = rotationMatrix(2,1);
    float r_33 = rotationMatrix(2,2);

    float theta = acos(0.5*(r_11+r_22+r_33-1));

    if (sin(theta)==0){
        if (cos(theta)==1){
            rotationVector(3) = 0;
            rotationVector(4) = 0;
            rotationVector(5) = 0;
        }
        if (cos(theta)==-1) {
            rotationVector(3) = theta;
            rotationVector(4) = theta;
            rotationVector(5) = theta;
        }
    }
    else{
        rotationVector(3) = theta* (r_32-r_23)/(2*sin(theta));
        rotationVector(4) = theta* (r_13-r_31)/(2*sin(theta));
        rotationVector(5) = theta* (r_21-r_12)/(2*sin(theta));
    }

    return rotationVector;
}

Matrix4d YoubotIkine::rotationVector_Matrix(VectorXd rotationVector){
    // Put u vector in a format of skew matrix
    Matrix3d rotationMatrix;
    rotationMatrix << 0, -rotationVector(5),rotationVector(4),
                     rotationVector(5),0,-rotationVector(3),
                     -rotationVector(4),rotationVector(3),0;

    rotationMatrix = rotationMatrix.exp();

    Matrix4d full_transformMat;

    full_transformMat(0,3) = rotationVector(0);
    full_transformMat(1,3) = rotationVector(1);
    full_transformMat(2,3) = rotationVector(2);
    full_transformMat(3,3) = 0;
    full_transformMat(3,2) = full_transformMat(3,3);
    full_transformMat(3,1) = full_transformMat(3,3);
    full_transformMat(3,0) = full_transformMat(3,3);

    full_transformMat.block(0,0,3,3) = rotationMatrix;

    return full_transformMat;
}

KDL::Frame YoubotIkine::poseMatrix_kdlFrame(Matrix4d pose){
    geometry_msgs::TransformStamped trans;
    trans.transform.translation.x = pose(0,3);
    trans.transform.translation.y = pose(1,3);
    trans.transform.translation.z = pose(2,3);

    // rotation matrix to quat
    double x,y,z,w;
    int matrixSize = 9;
    double r[matrixSize];

    for (int a=0 ; a < 3; a = a + 1){
        r[a]=pose(0,a);
    }
    for (int b=0 ; b < 3; b = b + 1){
        r[b+3]=pose(1,b);
    }
    for (int c=0 ; c < 3; c = c + 1){
        r[c+6]=pose(2,c);
    }

    // Mathematical manipulation

    double trace = r[0]+r[4]+r[8];

    if (trace > 0)
    {
        double denominator = sqrt(trace+1)*2; // denominator = 4 * qw
        x = (r[7]-r[5])/denominator;
        y = (r[2]-r[6])/denominator;
        z = (r[3]-r[1])/denominator;
        w = 0.25*denominator;
    }
    else if ( (r[0] > r[4]) & (r[0] > r[8]) )
    {
        double denominator = sqrt(1+r[0]-r[4]-r[8])*2; // denominator = 4 * qx
        x = 0.25*denominator;
        y = (r[1]+r[3])/denominator;
        z = (r[2]+r[6])/denominator;
        w = (r[7]-r[5])/denominator;
    }
    else if (r[4] > r[8])
    {
        double denominator = sqrt(1+r[4]-r[0]-r[8])*2; // denominator = 4 * qy
        x = (r[1]+r[3])/denominator;
        y = 0.25*denominator;
        z = (r[7]+r[5])/denominator;
        w = (r[2]-r[6])/denominator;
    }
    else
    {
        double denominator = sqrt(1+r[8]-r[0]-r[4])*2; // denominator = 4 * qz
        x = (r[2]+r[6])/denominator;
        y = (r[7]+r[5])/denominator;
        z = 0.25*denominator;
        w = (r[3]-r[1])/denominator;
    }

    trans.transform.rotation.x = x;
    trans.transform.rotation.y = y;
    trans.transform.rotation.z = z;
    trans.transform.rotation.w = w;


    KDL::Frame desiredPose = tf2::transformToKDL(trans);
    return desiredPose;

}

VectorXd YoubotIkine::pose_rotationVec(geometry_msgs::TransformStamped pose){
    // Quaternion to rotation vector
    VectorXd rotationVector(6);
    double q[4] = {pose.transform.rotation.w, pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z};
    // Mathematical manipulation
    rotationVector(0) = pose.transform.translation.x;
    rotationVector(1) = pose.transform.translation.y;
    rotationVector(2) = pose.transform.translation.z;

    double denominator = sqrt(1-q[0]*q[0]);
    double angle = 2*acos(q[0]);
    if (denominator < 0.001)
    {
        rotationVector(3) = angle*q[1];
        rotationVector(4) = angle*q[2];
        rotationVector(5) = angle*q[3];

    }
    else
    {
        rotationVector(3) = angle*q[1] / denominator;
        rotationVector(4) = angle*q[2] / denominator;
        rotationVector(5) = angle*q[3] / denominator;
    }
    return rotationVector;
}

Matrix4d YoubotIkine::pose_rotationMat(geometry_msgs::TransformStamped pose){
    // Quaternion to rotation matrix
    Matrix4d rotationMat;

    double qx,qy,qz,qw;

    qx = pose.transform.rotation.x;
    qy = pose.transform.rotation.y;
    qz = pose.transform.rotation.z;
    qw = pose.transform.rotation.w;

    rotationMat(0,0) = 1-2*pow(qy,2)-2*pow(qz,2);
    rotationMat(1,0) = 2*qx*qy+2*qz*qw;
    rotationMat(2,0) = 2*qx*qz-2*qy*qw;
    rotationMat(3,0) = 0;

    rotationMat(0,1) = 2*qx*qy-2*qz*qw;
    rotationMat(1,1) = 1-2*pow(qx,2)-2*pow(qz,2);
    rotationMat(2,1) = 2*qy*qz+2*qx*qw;
    rotationMat(3,1) = 0;

    rotationMat(0,2) = 2*qx*qz+2*qy*qw;
    rotationMat(1,2) = 2*qy*qz-2*qx*qw;
    rotationMat(2,2) = 1-2*pow(qx,2)-2*pow(qy,2);
    rotationMat(3,2) = 0;

    rotationMat(0,3) = pose.transform.translation.x;
    rotationMat(1,3) = pose.transform.translation.y;
    rotationMat(2,3) = pose.transform.translation.z;
    rotationMat(3,3) = 1;

    return rotationMat;
}

Matrix4d YoubotIkine::vectorQuat_rotationMat(VectorXd pose){
    // Quaternion to rotation matrix
    Matrix4d rotationMat;

    double qx,qy,qz,qw;

    qx = pose(3);
    qy = pose(4);
    qz = pose(5);
    qw = pose(6);

    rotationMat(0,0) = 1-2*pow(qy,2)-2*pow(qz,2);
    rotationMat(1,0) = 2*qx*qy+2*qz*qw;
    rotationMat(2,0) = 2*qx*qz-2*qy*qw;
    rotationMat(3,0) = 0;

    rotationMat(0,1) = 2*qx*qy-2*qz*qw;
    rotationMat(1,1) = 1-2*pow(qx,2)-2*pow(qz,2);
    rotationMat(2,1) = 2*qy*qz+2*qx*qw;
    rotationMat(3,1) = 0;

    rotationMat(0,2) = 2*qx*qz+2*qy*qw;
    rotationMat(1,2) = 2*qy*qz-2*qx*qw;
    rotationMat(2,2) = 1-2*pow(qx,2)-2*pow(qy,2);
    rotationMat(3,2) = 0;

    rotationMat(0,3) = pose(0);
    rotationMat(1,3) = pose(1);
    rotationMat(2,3) = pose(2);
    rotationMat(3,3) = 1;

    return rotationMat;
}

void YoubotIkine::publish_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory,int dt)
{

    int time_from_start = dt;//537230041;

    trajectory_msgs::JointTrajectory msg;

    msg.header.stamp = ros::Time::now();
    msg.joint_names.push_back("arm_joint_1");
    msg.joint_names.push_back("arm_joint_2");
    msg.joint_names.push_back("arm_joint_3");
    msg.joint_names.push_back("arm_joint_4");
    msg.joint_names.push_back("arm_joint_5");

    joint_trajectory.time_from_start.nsec = time_from_start;

    msg.points.push_back(joint_trajectory);

    this->publisher.publish(msg);
}