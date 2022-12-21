#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include "direct_kinematics.h"

#define _USE_MATH_DEFINES

typedef Eigen::Matrix<double, -1, 4> Vector4d;
typedef Eigen::Matrix<double, -1, JOINTS> JointConfiguration;

class Destination{
public:
    Eigen::Vector3d position;
    JointConfiguration joint_angles;
    Destination();
    void compute_inverse(const EndEffector& ee);
};

Destination::Destination(){
    position << Eigen::Vector3d::Zero();
}

void Destination::compute_inverse(const EndEffector& ee){
    Eigen::MatrixXd T06, P05;
    Eigen::Vector3d p60 = ee.position;
    Eigen::Vector4d cmp, p50;
    double phi1, phi2, p50_xy;
    double theta1[2], theta5[4];

    //populate T06
    cmp << 0,0,0,1;
    T06.resize(3,3);
    T06 << ee.orientation;
    T06.conservativeResize(Eigen::NoChange, T06.cols()+1);
    T06.col(T06.cols()-1) = p60;
    T06.conservativeResize(T06.rows()+1, Eigen::NoChange);
    T06.row(T06.rows()-1) = cmp;

    //theta1
    p50 << 0,0,-d[5],1;
    P05 = T06 * p50;
    phi1 = atan2(p50(1), p50(0));
    p50_xy = sqrt( pow(p50(0),2) + pow(p50(1),2) );
    phi2 = acos( d[3] / p50_xy );
    theta1[0] = phi1 + phi2 + M_PI_2;
    theta1[1] = phi1 - phi2 + M_PI_2;

    //theta5
    theta5[0] =  acos( ( p60(0)*sin(theta1[0]) - p60(1)*cos(theta1[0]) - d[3] ) / d[5] );
    theta5[1] = -acos( ( p60(0)*sin(theta1[0]) - p60(1)*cos(theta1[0]) - d[3] ) / d[5] );
    theta5[2] =  acos( ( p60(0)*sin(theta1[1]) - p60(1)*cos(theta1[1]) - d[3] ) / d[5] );
    theta5[3] = -acos( ( p60(0)*sin(theta1[1]) - p60(1)*cos(theta1[1]) - d[3] ) / d[5] );

    //theta6
}

#endif
