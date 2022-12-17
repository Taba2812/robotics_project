#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

#define JOINTS 6
#define DIM 4
#define ALPHA 1.57
#define CN 0
#define D 1

typedef Eigen::Matrix<double, DIM, DIM> Matrix4d;

class EndEffector{
public:
    Eigen::VectorXd position;
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
    position.resize(3,1);
}

void EndEffector::compute_direct(const Eigen::VectorXd& q){
    Matrix4d T0, T1, T2, T3, T4, T5, TF;

    T(T0, q, 0);
    T(T1, q, 1);
    T(T2, q, 2);
    T(T3, q, 3);
    T(T4, q, 4);
    T(T5, q, 5);

    TF = T0*T1*T2*T3*T4*T5;

    this->orientation = TF.block<3,3>(0,0);
    this->position = TF.block<3,1>(0,3);

}

#endif
