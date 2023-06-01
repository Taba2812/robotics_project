#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include "ur5.h"

class EndEffector{
private:
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
public:
    EndEffector();
    Eigen::Vector3d getPosition() const;
    Eigen::Matrix3d getOrientation() const;
    void setPosition(Eigen::Vector3d& pos);
    void setOrientation(Eigen::Matrix3d& angle);
    void computeDirect(const Eigen::VectorXd& q);
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

Eigen::Vector3d EndEffector::getPosition() const{
    return this->position;
}

void EndEffector::setPosition(Eigen::Vector3d& p){
    position = p;
}

Eigen::Matrix3d EndEffector::getOrientation() const{
    return this->orientation;
}

void EndEffector::setOrientation(Eigen::Matrix3d& o){
    orientation = o;
}

void EndEffector::computeDirect(const Eigen::VectorXd &q){
    Matrix4d T1, T2, T3, T4, T5, T6, T60;

    T(T1, q[0], 0);
    T(T2, q[1], 1);
    T(T3, q[2], 2);
    T(T4, q[3], 3);
    T(T5, q[4], 4);
    T(T6, q[5], 5);

    T60 = T1*T2*T3*T4*T5*T6;

    this->orientation = T60.block<3,3>(0,0);
    this->position = T60.block<3,1>(0,3);
}

#endif
