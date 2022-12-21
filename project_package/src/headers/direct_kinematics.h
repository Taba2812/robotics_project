#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

#define JOINTS 6
#define DIM 4

//these are stand-in values, I have to figure them out from a kinematic diagram of the ur5, may vary joint to joint
#define R90 1.57  //90 degrees in radians

typedef Eigen::Matrix<double, DIM, DIM> Matrix4d;

double alpha[JOINTS] = {R90,R90,R90,R90,R90,R90};   //angles (?)
double cn[JOINTS] = {0,0,0,0,0,0};                  //common normal
double d[JOINTS] = {1,1,1,1,1,1};                   //distance between axes

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
    m << cos(q[index]), -sin(q[index])*cos(alpha[index]), sin(q[index])*sin(alpha[index]) , cn[index]*cos(q[index]),
         sin(q[index]), cos(q[index])*cos(alpha[index]) , -cos(q[index]*sin(alpha[index])), cn[index]*sin(q[index]),
         0            , sin(alpha[index])               , cos(alpha[index])               , d[index],
         0            , 0                               , 0                               , 1;
}

EndEffector::EndEffector(){
    position << Eigen::Vector3d::Zero();
    orientation << Eigen::Matrix3d::Zero();
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
