#ifndef __DIFFERENTIAL_KINEMATICS_H__
#define __DIFFERENTIAL_KINEMATICS_H__

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include "direct_kinematics.h"

using namespace std;

class Motion{
public:
    Motion(const EndEffector& ee);
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;
    Eigen::VectorXd compute_differential(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;
private:
    const EndEffector& ee;
};

Motion::Motion(const EndEffector& ee) : ee(ee){
}

//just a draft, will have to think about how to use the compute_direct() function...
Eigen::MatrixXd Motion::jacobian(const Eigen::VectorXd& q) const{
    //gets the transformation matrix for each joint
    Matrix4d T10, T21, T32, T43, T54, T65;
    
    T(T10, q[0], 0);
    T(T21, q[1], 1);
    T(T32, q[2], 2);
    T(T43, q[3], 3);
    T(T54, q[4], 4);
    T(T65, q[5], 5);

    vector<Eigen::Matrix4d> transformations = {T10, T21, T32, T43, T54, T65};

    // Calculate the position and orientation of the end effector
    Eigen::Vector3d pos = transformations.back().block<3, 1>(0, 3);
    //Eigen::Matrix3d rot = transformations.back().block<3, 3>(0, 0);

    //m=6 because the robot operates in 3 dimensions, n=6 because we have 6 joints total
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);

    for (int i = 0; i < 6; i++){
        Eigen::Vector3d z_i = transformations[i].block<3, 1>(0, 2);     //unit vector
        Eigen::Vector3d p_i = transformations[i].block<3, 1>(0, 3);     //joint position
        Eigen::Vector3d w_i = z_i;                                      //angular velocity of the joint
        Eigen::Vector3d v_i = w_i.cross(pos - p_i);                     //linear velocity of the joint
        J.block<3, 1>(0, i) = v_i;
        J.block<3, 1>(3, i) = w_i;
    }

    return J;
}

Eigen::VectorXd Motion::compute_differential(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const{
    Eigen::MatrixXd J = jacobian(q);
    return J * dq;
}

#endif