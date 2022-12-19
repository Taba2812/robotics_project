#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

#define JOINTS 6
#define DIM 4

//these are stand-in values, I have to figure them out from a kinematic diagram of the ur5, may vary joint to joint
#define ALPHA 1.57  //90 degrees in radians
#define CN 0        //common normal
#define D 1         //distance between axes for DH parameters

typedef Eigen::Matrix<double, DIM, DIM> Matrix4d;

class EndEffector{
public:
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
    EndEffector();
    void compute_direct(const Eigen::VectorXd& q);
    friend std::ostream& operator<<(std::ostream& os, const EndEffector& ef);
};

std::ostream& operator<<(std::ostream& os, const EndEffector& ef){
    return os << ef.position;
}

void T(Matrix4d& m, const Eigen::VectorXd& q, int index){
    m << cos(q[index]), -sin(q[index])*cos(ALPHA), sin(q[index])*sin(ALPHA) , CN*cos(q[index]),
         sin(q[index]), cos(q[index])*cos(ALPHA) , -cos(q[index]*sin(ALPHA)), CN*sin(q[index]),
         0            , sin(ALPHA)               , cos(ALPHA)               , D,
         0            , 0                        , 0                        , 1;
}

EndEffector::EndEffector(){
    position << 0,0,0;
    orientation << 0,0,0,0,0,0,0,0,0;
}

void EndEffector::compute_direct(const Eigen::VectorXd& q){
    Matrix4d T10, T21, T32, T43, T54, T65, T06;

    T(T10, q, 0);
    T(T21, q, 1);
    T(T32, q, 2);
    T(T43, q, 3);
    T(T54, q, 4);
    T(T65, q, 5);

    T06 = T10*T21*T32*T43*T54*T65;

    this->orientation = T06.block<3,3>(0,0);
    this->position = T06.block<3,1>(0,3);

}

#endif
