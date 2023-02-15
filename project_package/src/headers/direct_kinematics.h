#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include "ur5.h"

class EndEffector{
private:
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
public:
    EndEffector();
    Eigen::Vector3d get_position() const;
    Eigen::Matrix3d get_orientation() const;
    void set_position(Eigen::Vector3d& pos);
    void set_orientation(Eigen::Matrix3d& angle);
    void compute_direct(const Eigen::VectorXd& q);
    friend std::ostream& operator<<(std::ostream& os, const EndEffector& ef);
};

std::ostream& operator<<(std::ostream& os, const EndEffector& ef){
    return os << ef.position;
}

void T(Matrix4d& m, double q, int index){
    m << cos(q), -sin(q)*cos(alpha[index]), sin(q)*sin(alpha[index]) , cn[index]*cos(q),
         sin(q), cos(q)*cos(alpha[index]) , -cos(q)*sin(alpha[index]), cn[index]*sin(q),
         0     , sin(alpha[index])        , cos(alpha[index])        , d[index],
         0     , 0                        , 0                        , 1;
}

EndEffector::EndEffector(){
    position << Eigen::Vector3d::Zero();
    orientation << Eigen::Matrix3d::Zero();
}

Eigen::Vector3d EndEffector::get_position() const{
    return this->position;
}

void EndEffector::set_position(Eigen::Vector3d& pos){
    position = pos;
}

Eigen::Matrix3d EndEffector::get_orientation() const{
    return this->orientation;
}

void EndEffector::set_orientation(Eigen::Matrix3d& angle){
    orientation = angle;
}

void EndEffector::compute_direct(const Eigen::VectorXd& q){
    Matrix4d T10, T21, T32, T43, T54, T65, T60;

    T(T10, q[0], 0);
    T(T21, q[1], 1);
    T(T32, q[2], 2);
    T(T43, q[3], 3);
    T(T54, q[4], 4);
    T(T65, q[5], 5);

    T60 = T10*T21*T32*T43*T54*T65;

    this->orientation = T60.block<3,3>(0,0);
    this->position = T60.block<3,1>(0,3);
}

#endif
