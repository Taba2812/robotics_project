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
    void setPosition(Eigen::Vector3d p);
    void setOrientation(Eigen::Matrix3d& o);
    void computeDirect(const Eigen::VectorXd& q);
    friend std::ostream& operator<<(std::ostream& os, const EndEffector& ef);
};

std::ostream& operator<<(std::ostream& os, const EndEffector& ee){
    return os << ee.position;
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

void EndEffector::setPosition(Eigen::Vector3d p){
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

    T(T1, q(0), 0);
    T(T2, q(1), 1);
    T(T3, q(2), 2);
    T(T4, q(3), 3);
    T(T5, q(4), 4);
    T(T6, q(5), 5);

    T60 = T1*T2*T3*T4*T5*T6;

    this->orientation = T60.block<3,3>(0,0);
    this->position = T60.block<3,1>(0,3);
}

#endif


/*

Eigen::Matrix4d computeDirect(const Eigen::VectorXd& jointAngles)
{

    // Extract joint angles
    double q1 = jointAngles(0);
    double q2 = jointAngles(1);
    double q3 = jointAngles(2);
    double q4 = jointAngles(3);
    double q5 = jointAngles(4);
    double q6 = jointAngles(5);

    // Compute the individual transformation matrices
    Eigen::Matrix4d T1, T2, T3, T4, T5, T6, transformationMatrix;
    
    T1 << std::cos(q1), -std::sin(q1), 0, 0,
        std::sin(q1), std::cos(q1), 0, 0,
        0, 0, 1, d1,
        0, 0, 0, 1;

    T2 << std::cos(q2), -std::sin(q2), 0, 0,
        0, 0, -1, 0,
        std::sin(q2), std::cos(q2), 0, 0,
        0, 0, 0, 1;

    T3 << std::cos(q3), -std::sin(q3), 0, a2,
        std::sin(q3), std::cos(q3), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T4 << std::cos(q4), -std::sin(q4), 0, a3,
        0, 0, -1, -d4,
        std::sin(q4), std::cos(q4), 0, 0,
        0, 0, 0, 1;

    T5 << std::cos(q5), -std::sin(q5), 0, 0,
        0, 0, 1, -d5,
        -std::sin(q5), -std::cos(q5), 0, 0,
        0, 0, 0, 1;

    T6 << std::cos(q6), -std::sin(q6), 0, 0,
        0, 0, -1, -d6,
        std::sin(q6), std::cos(q6), 0, 0,
        0, 0, 0, 1;

    // Compute the transformation matrix from the base to the end-effector
    transformationMatrix = T1 * T2 * T3 * T4 * T5 * T6;

    return transformationMatrix;
}

*/