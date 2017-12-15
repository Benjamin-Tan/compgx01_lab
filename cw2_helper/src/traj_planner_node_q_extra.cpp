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
MatrixXf traj_points = MatrixXf::Zero(1,5);
MatrixXf q_free = MatrixXf::Zero(3,5);

struct obstacle{
    // 0:x, 1:y, 2:z
    float centre[3];

    float length[3];
};
obstacle box1,box2,box3,box4,cyl0,cyl1;

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;

    std::vector<std::string> topics;
    MatrixXd p;
    p.resize(5,3);

    VectorXd desired_joint_value;

    bag.open("/home/benjamintan/catkin_ws/src/compgx01_lab/cw2_helper/bags/data_extra.bag", rosbag::bagmode::Read);
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

                    }
                    countMessage++;

                }

    bag.close();

    return p;

}

void assignObstacle_properties(YoubotIkine youbot_kine){
    // assign obstacle position x,y,z into each class
    for (int i = 0; i<3; i++){
        box1.centre[i] = youbot_kine.obstacle_position_extra(0,i);
        box2.centre[i] = youbot_kine.obstacle_position_extra(1,i);
        cyl0.centre[i] = youbot_kine.obstacle_position_extra(2,i);
        cyl1.centre[i] = youbot_kine.obstacle_position_extra(3,i);
        box3.centre[i] = youbot_kine.obstacle_position_extra(4,i);
        box4.centre[i] = youbot_kine.obstacle_position_extra(5,i);
    }

    // assign length properties into each obstacle
    box1.length[0] = 0.194033;
    box1.length[1] = 0.077038;
    box1.length[2] = 0.355229;

    box2.length[0] = 0.167065;
    box2.length[1] = 0.112100;
    box2.length[2] = 0.247538;

    box3.length[0] = 0.233423;
    box3.length[1] = 0.376435;
    box3.length[2] = 0.227683;

    box4.length[0] = 0.320579;
    box4.length[1] = 0.506932;
    box4.length[2] = 0.064487;

    cyl0.length[0] = 0.05; //radius
    cyl0.length[1] = 0.22; //length
    cyl0.length[2] = 0;

    cyl1.length[0] = 0.06; //radius
    cyl1.length[1] = 0.14; //length
    cyl1.length[2] = 0;

}

bool checkCollision_node(VectorXd joints_random, YoubotIkine youbot_kine){
    // return true if collide
    MatrixXd T_random = youbot_kine.forward_kinematics(joints_random,4);
    Vector3d pose_random = T_random.block(0,3,3,1);
    bool collideState = 0;
    // check every obstacle
    // box1
    // rotate the pose to box1's frame
    VectorXd rotated_pose_random = VectorXd::Zero(7);
    rotated_pose_random << pose_random, youbot_kine.obstacle_position_extra(0,3),youbot_kine.obstacle_position_extra(0,4),\
                            youbot_kine.obstacle_position_extra(0,5),youbot_kine.obstacle_position_extra(0,6);
    Matrix4d rotated_pose_random_mat = youbot_kine.vectorQuat_rotationMat(rotated_pose_random);
    Vector3d new_pose_random = rotated_pose_random_mat.block(0,0,3,3).inverse() * pose_random;
    if (abs(new_pose_random(0)-box1.centre[0])<= box1.length[0]/2 && \
        abs(new_pose_random(1)-box1.centre[1])<= box1.length[1]/2 && \
        abs(new_pose_random(2)-box1.centre[2])<= box1.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box2.centre[0])<= box2.length[0]/2 && \
        abs(pose_random(1)-box2.centre[1])<= box2.length[1]/2 && \
        abs(pose_random(2)-box2.centre[2])<= box2.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box3.centre[0])<= box3.length[0]/2 && \
        abs(pose_random(1)-box3.centre[1])<= box3.length[1]/2 && \
        abs(pose_random(2)-box3.centre[2])<= box3.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box4.centre[0])<= box4.length[0]/2 && \
        abs(pose_random(1)-box4.centre[1])<= box4.length[1]/2 && \
        abs(pose_random(2)-box4.centre[2])<= box4.length[2]/2)
        collideState = 1;
    else
        collideState = 0;
    // cylinder
    // (x-x_o)^2 + (y-y_o)^2 <= r^2 + (z-z_o)^2
    if ((pow(pose_random(0)-cyl0.centre[0],2) + pow(pose_random(1)-cyl0.centre[1],2)) \
            <= (pow(cyl0.length[0],2) + pow(pose_random(2)-cyl0.centre[2],2)) && \
        abs(pose_random(2)-cyl0.centre[2])<= cyl0.length[1]/2)
        collideState = 1;
    else
        collideState = 0;

    if ((pow(pose_random(0)-cyl1.centre[0],2) + pow(pose_random(1)-cyl1.centre[1],2)) \
            <= (pow(cyl1.length[0],2) + pow(pose_random(2)-cyl1.centre[2],2)) && \
        abs(pose_random(2)-cyl1.centre[2])<= cyl1.length[1]/2)
        collideState = 1;
    else
        collideState = 0;

    return collideState;

}

bool checkCollision_node_cart(VectorXd joints_cart, YoubotIkine youbot_kine){
    // return true if collide
    Vector3d pose_random = joints_cart;
    bool collideState = 0;
    // check every obstacle
    // box1
    // rotate the pose to box1's frame
    VectorXd rotated_pose_random = VectorXd::Zero(7);
    rotated_pose_random << pose_random, youbot_kine.obstacle_position_extra(0,3),youbot_kine.obstacle_position_extra(0,4),\
                            youbot_kine.obstacle_position_extra(0,5),youbot_kine.obstacle_position_extra(0,6);
    Matrix4d rotated_pose_random_mat = youbot_kine.vectorQuat_rotationMat(rotated_pose_random);

    Vector3d new_pose_random = rotated_pose_random_mat.block(0,0,3,3).inverse() * pose_random;
    if (abs(new_pose_random(0)-box1.centre[0])<= box1.length[0]/2 && \
        abs(new_pose_random(1)-box1.centre[1])<= box1.length[1]/2 && \
        abs(new_pose_random(2)-box1.centre[2])<= box1.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box2.centre[0])<= box2.length[0]/2 && \
        abs(pose_random(1)-box2.centre[1])<= box2.length[1]/2 && \
        abs(pose_random(2)-box2.centre[2])<= box2.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box3.centre[0])<= box3.length[0]/2 && \
        abs(pose_random(1)-box3.centre[1])<= box3.length[1]/2 && \
        abs(pose_random(2)-box3.centre[2])<= box3.length[2]/2)
        collideState = 1;
    else
        collideState = 0;

    if (abs(pose_random(0)-box4.centre[0])<= box4.length[0]/2 && \
        abs(pose_random(1)-box4.centre[1])<= box4.length[1]/2 && \
        abs(pose_random(2)-box4.centre[2])<= box4.length[2]/2)
        collideState = 1;
    else
        collideState = 0;
    // cylinder
    // (x-x_o)^2 + (y-y_o)^2 <= r^2 + (z-z_o)^2
    if ((pow(pose_random(0)-cyl0.centre[0],2) + pow(pose_random(1)-cyl0.centre[1],2)) \
            <= (pow(cyl0.length[0],2) + pow(pose_random(2)-cyl0.centre[2],2)) && \
        abs(pose_random(2)-cyl0.centre[2])<= cyl0.length[1]/2)
        collideState = 1;
    else
        collideState = 0;

    if ((pow(pose_random(0)-cyl1.centre[0],2) + pow(pose_random(1)-cyl1.centre[1],2)) \
            <= (pow(cyl1.length[0],2) + pow(pose_random(2)-cyl1.centre[2],2)) && \
        abs(pose_random(2)-cyl1.centre[2])<= cyl1.length[1]/2)
        collideState = 1;
    else
        collideState = 0;

    return collideState;

}

bool pairCompare(std::pair<float,float> i, std::pair<float,float> j){
    return i.second < j.second;
}

void traj_q_extra (MatrixXd checkpoint, YoubotIkine youbot_kine, YoubotKDL youbot_kdl)
{
    // Do something
    // Random Sampling Roadmap (Probabilistic Road Map) ===============================
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution1(0.0,5.899);
    std::uniform_real_distribution<double> distribution2(0.0,2.705);
    std::uniform_real_distribution<double> distribution3(-5.1836,0.0);
    std::uniform_real_distribution<double> distribution4(0.0,3.578);
    std::uniform_real_distribution<double> distribution5(0.0,5.8469);

    int i = 0;
    int no_sampling = 10000;
    int updateSize=0;
    float joint1,joint2,joint3,joint4,joint5;
    VectorXd joints_random = VectorXd::Zero(5);

    while (q_free.rows()<no_sampling){
        joint1 = distribution1(generator);
        joint2 = distribution2(generator);
        joint3 = distribution3(generator);
        joint4 = distribution4(generator);
        joint5 = distribution5(generator);

        joints_random << joint1,joint2,joint3,joint4,joint5;

        if (checkCollision_node(joints_random,youbot_kine)){
            // ignore this configuration, as it lies within obstacle
        }
        else{
            // store the current joint configuration as q_free
            if (i>2)
                q_free.conservativeResize(i+1,5); // increase the matrix size

            q_free.block(i,0,1,5) = joints_random.transpose().cast<float>();
            std::cout<<i<<"\n"<<q_free<<"\n"<<std::endl;
            i=i+1;
        }

    }// end PRM sampling

    // find k-nearest neighbour and connect edges without colliding obstacle ================
    int k = 10; // nearest neighbour

    std::map <float,VectorXf> distanceMap, distanceMap_free; //key: distance, value: index(q_current, q_other)
    std::map <float,VectorXf> knnMap[no_sampling];
    VectorXf q_index_concatenate(2); // index(q_current) + index(q_other)
    float dist;

    // loop for each q (joint configuration)
    for (int q=0; q<no_sampling; q++){

        distanceMap.clear(); // initialise to zero every loop
        distanceMap_free.clear();

        for (int q_other = 0; q_other<no_sampling; q_other++){
            if (q==q_other)
                continue; // skip the specific step
            else{
                q_index_concatenate << q,q_other;
                Matrix4d current_free = youbot_kine.forward_kinematics(q_free.row(q).transpose().cast<double>(),4);
                Matrix4d other_free = youbot_kine.forward_kinematics(q_free.row(q_other).transpose().cast<double>(),4);
                dist = (current_free.block(0,3,3,1) - other_free.block(0,3,3,1)).norm();

                distanceMap.insert(std::pair <float,VectorXf> (dist,q_index_concatenate));

            }
            std::cout<<"q: "<<q<<"  qOther: "<<q_other<<std::endl;
            std::cout<<"\n"<<std::endl;

        }

        std::map<float,VectorXf>::iterator itr;

        // check whether the edges collides with obstacle, if true, discard that edge
        for (itr = distanceMap.begin(); itr!= distanceMap.end() ; ++itr){
            std::cout<<"not free:"<< itr->first << '\t'<<itr->second<<'\n' <<std::endl;
            // interpolate 10 points between the 2 joint configuration
            int current_node_index = itr->second(0);
            int final_node_index = itr->second(1);
            int point_interpolated = 10;
            int collideState = 0;

            VectorXf current_node = q_free.row(current_node_index);
            VectorXf final_node = q_free.row(final_node_index);
            VectorXf node_t = VectorXf::Zero(5);

            // loop over each interpolation
            for (int p=0; p<point_interpolated; ++p){
                // loop through each joints
                node_t.setZero();

                for (int m=0; m<5; ++m){
                    node_t(m) = current_node(m) + 0.1*p*(final_node(m)-current_node(m));
                }
                if (checkCollision_node(node_t.cast<double>(),youbot_kine)){
                    collideState = 1;
                }else{
                    collideState = 0;
                }

            }
            std::cout<<"State: "<<collideState<<std::endl;
            if (collideState){
                // do not store into distanceMap_free
            }
            else{
                distanceMap_free.insert(std::pair <float,VectorXf> (itr->first,itr->second));
            }

        }
        // store the k neighbours of current node with collision free edges
        for (itr = distanceMap_free.begin(); itr!= std::next(distanceMap_free.begin(),k); ++itr){
            std::cout<<"free"<< itr->first << '\t'<<itr->second<<'\n' <<std::endl;
            knnMap[q].insert(std::pair <float,VectorXf> (itr->first,itr->second));
        }

    }// end k nearest neighbours and connecting edges between nodes

    // Compute the shortest path using Dijkstra's algorithm ==========================================
    std::cout<<checkpoint<<std::endl;
    int countPoints = checkpoint.rows();
    VectorXf init_q_cart = VectorXf::Zero(3);
    VectorXf final_q_cart = VectorXf::Zero(3);

    // from checkpoint to checkpoint
    for (int cPoint = 0; cPoint<countPoints; ++cPoint){
        Vector3f desiredPose, desiredPose_old; // only the x,y,z cartesian coordinate

        desiredPose << checkpoint(cPoint, 0), checkpoint(cPoint, 1), checkpoint(cPoint, 2);

        // for the initial step only
        if (cPoint==0){
            init_q_cart = youbot_kine.forward_kinematics(youbot_kine.current_joint_position,4).block(0,3,3,1).cast<float>();
            final_q_cart = desiredPose;
        }
        else{
            init_q_cart = desiredPose_old;
            final_q_cart = desiredPose;
        }

        /* find the nearest neighbour for init_q and final_q */

        //key: distance, value: index(q_current, q_other)
        std::map <float,VectorXf> distanceMap_init_q, distanceMap_init_q_free, distanceMap_final_q, distanceMap_final_q_free ;
        std::map <float,VectorXf> init_q_nn, final_q_nn;

        distanceMap_init_q.clear(); // initialise to zero every loop
        distanceMap_init_q_free.clear();
        distanceMap_final_q.clear();
        distanceMap_final_q_free.clear();
        init_q_nn.clear();
        final_q_nn.clear();

        for (int q_other = 0; q_other<no_sampling; q_other++){

            q_index_concatenate << -1,q_other; // -1: init point, -2: final point
            Matrix4f other_free = youbot_kine.forward_kinematics(q_free.row(q_other).transpose().cast<double>(),4).cast<float>();
            dist = (init_q_cart - other_free.block(0,3,3,1)).norm();
            distanceMap_init_q.insert(std::pair <float,VectorXf> (dist,q_index_concatenate));

            std::cout<<"Initq: "<<dist<<", "<<q_other<<"\n"<<std::endl;

            q_index_concatenate << -2,q_other;
            dist = (final_q_cart - other_free.block(0,3,3,1)).norm();
            std::cout<<"Finaq: "<<dist<<", "<<q_other<<"\n"<<std::endl;
            distanceMap_final_q.insert(std::pair <float,VectorXf> (dist,q_index_concatenate));

        }

        std::map<float,VectorXf>::iterator itr;
        /* for init_q nearest neighbour */
        // check whether the edges collides with obstacle, if true, discard that edge
        for (itr = distanceMap_init_q.begin(); itr!= distanceMap_init_q.end() ; ++itr){
            std::cout<<"not free:"<< itr->first << '\t'<<itr->second<<'\n' <<std::endl;
            // interpolate 10 points between the 2 joint configuration
            int final_node_index = itr->second(1);
            int point_interpolated = 10;
            int collideState = 0;

            VectorXf current_node = init_q_cart;
            Matrix4f final_mat = youbot_kine.forward_kinematics(q_free.row(final_node_index).transpose().cast<double>(),4).cast<float>();
            VectorXf final_node = final_mat.block(0,3,3,1);
            VectorXf node_t = VectorXf::Zero(3);

            // loop over each interpolation
            for (int p=0; p<point_interpolated; ++p){
                // loop through each joints
                node_t.setZero();

                for (int m=0; m<3; ++m){
                    node_t(m) = current_node(m) + 0.1*p*(final_node(m)-current_node(m));
                }
                if (checkCollision_node_cart(node_t.cast<double>(),youbot_kine)){
                    collideState = 1;
                }else {
                    collideState = 0;
                }
            }
            if (collideState){
                // do not store into distanceMap_free
            }
            else{
                distanceMap_init_q_free.insert(std::pair <float,VectorXf> (itr->first,itr->second));
            }
        }
        // store the nearest neighbours of current node with collision free edges
        init_q_nn.insert(std::pair <float,VectorXf> (distanceMap_init_q_free.begin()->first,distanceMap_init_q_free.begin()->second));

        /* for final_q nearest neighbour */
        // check whether the edges collides with obstacle, if true, discard that edge
        for (itr = distanceMap_final_q.begin(); itr!= distanceMap_final_q.end() ; ++itr){
            std::cout<<"not free:"<< itr->first << '\t'<<itr->second<<'\n' <<std::endl;
            // interpolate 10 points between the 2 joint configuration
            int final_node_index = itr->second(1);
            int point_interpolated = 10;
            int collideState = 0;

            VectorXf current_node = init_q_cart;
            Matrix4f final_mat = youbot_kine.forward_kinematics(q_free.row(final_node_index).transpose().cast<double>(),4).cast<float>();
            VectorXf final_node = final_mat.block(0,3,3,1);
            VectorXf node_t = VectorXf::Zero(3);

            // loop over each interpolation
            for (int p=0; p<point_interpolated; ++p){
                // loop through each joints
                node_t.setZero();

                for (int m=0; m<3; ++m){
                    node_t(m) = current_node(m) + 0.1*p*(final_node(m)-current_node(m));
                }
                if (checkCollision_node_cart(node_t.cast<double>(),youbot_kine)){
                    collideState = 1;
                }else{
                    collideState = 0;
                }
            }
            std::cout<<"\n State: "<<collideState<<std::endl;
            if (collideState){
                // do not store into distanceMap_free
            }
            else{
                distanceMap_final_q_free.insert(std::pair <float,VectorXf> (itr->first,itr->second));
            }

        }
        // store the nearest neighbours of current node with collision free edges
        final_q_nn.insert(std::pair <float,VectorXf> (distanceMap_final_q_free.begin()->first,distanceMap_final_q_free.begin()->second));

        // end q_init q_final nearest neighbours and connecting edges between nodes =======================
        std::cout<<"init nn: \n"<< init_q_nn.begin()->first << '\t'<<init_q_nn.begin()->second<<'\n' <<std::endl;
        std::cout<<"final nn: \n"<<final_q_nn.begin()->first << '\t'<<final_q_nn.begin()->second<<'\n' <<std::endl;

        // Main Dijkstra's Algorithm =====================================
        std::map <float,float> unvisited_node; //key:current_node_index  value: distance or cost,
        std::map <float,float> updated_node;// key:current_node_index value: its parent
        unvisited_node.clear();
        updated_node.clear();

        // assign infinity unary cost for each node, except for starting node
        for (int q=0; q<no_sampling; q++){
            if (q==init_q_nn.begin()->second(1)){
                unvisited_node.insert(std::pair <float,float> (q, init_q_nn.begin()->first));
            }else{
                unvisited_node.insert(std::pair <float,float> (q, std::numeric_limits<float>::infinity()));
            }
        }

        // loop till all the node are visited
        while (!unvisited_node.empty()){
            // find the corresponding key from minimum value from the map
            std::pair<float,float> min = *std::min_element(unvisited_node.begin(),unvisited_node.end(),pairCompare);

            std::cout<<"Minimum!!: "<<min.first<<","<<min.second<<"\n"<<std::endl;
            int u = min.first; // current node index
            float cost_current = min.second;
            unvisited_node.erase(u); //remove the current entry

            for (itr = knnMap[u].begin(); itr!=knnMap[u].end(); ++itr){
                float cost_neighbour = itr->first;
                int v = itr->second(1); //neighbour index of current node

                if (cost_current + cost_neighbour < unvisited_node.find(v)->second){
                    unvisited_node[v] = cost_current + cost_neighbour;
                    updated_node.insert(std::pair <float,float> (v,u)); // store the current node with respective parent
                }

                std::map<float,float>::iterator itr11;
                for (itr11 = unvisited_node.begin(); itr11!=unvisited_node.end(); ++itr11){
                    std::cout<< itr11->first << '\t'<<itr11->second<<'\n' <<std::endl;
                }
                std::cout<<"\n"<<std::endl;

            }

        }
        std::map<float,float>::iterator itr11;
        for (itr11 = updated_node.begin(); itr11!=updated_node.end(); ++itr11){
            std::cout<<"node and parent: \n"<< itr11->first << '\t'<<itr11->second<<'\n' <<std::endl;
        }
        // backward pass to find the shortest path, and store the path into global variable
        std::vector<int> pathSeq;//path sequence from start to end
        pathSeq.clear();
        std::vector<int>::iterator count;

        int startIndex = init_q_nn.begin()->second(1);
        int endIndex = final_q_nn.begin()->second(1);
        int findIndex = endIndex;

        count = pathSeq.begin();
        count = pathSeq.insert(count,endIndex);

        while (findIndex!=startIndex){
            findIndex = updated_node.find(endIndex)->second;
            std::cout<<findIndex<<std::endl;
            count = pathSeq.begin();
            pathSeq.insert(count,findIndex);
            endIndex = findIndex;
        }

        std::cout<<"Path: "<<std::endl;
        for (count = pathSeq.begin(); count!=pathSeq.end(); ++count){
            std::cout<<*count<<std::endl;
        }
        std::cout<<pathSeq.at(0)<<std::endl;
        std::cout<<"\n"<<std::endl;

        if ((pathSeq.size()+updateSize)>traj_points.rows()){
            traj_points.conservativeResize(updateSize + pathSeq.size() , 5);
        }

        for (int i = 0 ; i<pathSeq.size(); ++i){

            traj_points.block(updateSize+i,0,1,5) = q_free.row(pathSeq.at(i));
        }
//        traj_points.block(cPoint,0,)
        std::cout<<traj_points<<std::endl;

        updateSize = traj_points.rows();
        // for previous position
        desiredPose_old = desiredPose;

    }

//    for (int j=0; j<10;++j){
//        std::cout<<"j: "<<j<<"\n"<<std::endl;
//        std::map<float,VectorXf>::iterator itr;
//        for (itr = knnMap[j].begin(); itr!=knnMap[j].end(); ++itr){
//            std::cout<< itr->first << '\t'<<itr->second<<'\n' <<std::endl;
//        }
//        std::cout<<"\n"<<std::endl;
//    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_q_extra");

    YoubotIkine youbot_kine;
    YoubotKDL youbot_kdl;

    youbot_kine.init();
    youbot_kdl.init();


    MatrixXd check_point_matrix = get_checkpoint();
    std::cout<<check_point_matrix<<"\n"<<std::endl;

    // spinOnce cannot get the obstacle position
    for (int i = 0; i<3;i++){
        ros::spinOnce();
        ros::Duration(0.3).sleep();
        std::cout<<i<<"...Obstacle Position\n"<<youbot_kine.obstacle_position_extra<<std::endl;
    }

    assignObstacle_properties(youbot_kine);

    traj_q_extra(check_point_matrix,youbot_kine,youbot_kdl);
    int countPoints=traj_points.rows();
    int cPoints = 0;
    std::cout<<countPoints<<std::endl;
    while (ros::ok())
    {
        for (cPoints=0;cPoints<countPoints;cPoints++) {
            std::cout<<"start "<< cPoints <<"\n"<<std::endl;

            traj_pt.positions.clear();
            for (int i=0; i<5; i++){
                traj_pt.positions.push_back(traj_points(cPoints,i));
            }
            double dt = 0.8;//traj_points(cPoints,6);
            youbot_kine.publish_trajectory(traj_pt,dt*1e9);

            ros::Duration(dt).sleep();
            std::cout<<"after publish\n"<<std::endl;
            ros::spinOnce();
        }
        //sleep(1);
    }

    return 0;
}
