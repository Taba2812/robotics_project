#ifndef __DIRECT_KINEMATICS_H__
#define __DIRECT_KINEMATICS_H__

#include <iostream>
#include <Eigen/Dense>

#define JOINTS 6
#define DIM 4

class EndEffector{
public:
    Eigen::VectorXd pose;
    EndEffector();
    void compute_direct(const Eigen::VectorXd& q);
    friend std::ostream& operator<<(std::ostream& os, const EndEffector& ef);
};

std::ostream& operator<<(std::ostream& os, const EndEffector& ef){
    return os << ef.pose;
}

EndEffector::EndEffector(){
    pose.resize(6,1);
}

void EndEffector::compute_direct(const Eigen::VectorXd& q){
    Eigen::MatrixXd m1(DIM, DIM);
    Eigen::MatrixXd m2(DIM, DIM);
    Eigen::MatrixXd m3(DIM, DIM);
    Eigen::MatrixXd m4(DIM, DIM);
    Eigen::MatrixXd m5(DIM, DIM);
    Eigen::MatrixXd m6(DIM, DIM);
    Eigen::MatrixXd m7(DIM, DIM);
}

#endif
