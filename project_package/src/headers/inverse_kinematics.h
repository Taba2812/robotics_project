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
    Eigen::MatrixXd T60, T06, T61, T41;
    Eigen::Matrix4d T65, T54, T10;
    Eigen::Vector3d pos, X06, Y06;
    Eigen::Vector4d cmp, tmp, P50, P31;
    double phi, psi, R, n;
    double th1[2], th5[4], th6[4];

    pos = ee.position;

    //T60(th1,th2,th3,th4,th5,th6) = T10(th1)*T21(th2)*T32(th3)*T43(th4)*T54(th5)*T65(th6)
    cmp << 0,0,0,1;
    T60.resize(3,3);
    T60 << ee.orientation;
    T60.conservativeResize(Eigen::NoChange, T60.cols()+1);
    T60.col(T60.cols()-1) = ee.position;
    T60.conservativeResize(T06.rows()+1, Eigen::NoChange);
    T60.row(T60.rows()-1) = cmp;

    //theta1
    tmp << 0,0,-d[5],1;
    P50 = T60 * tmp;
    phi = atan2(P50(1), P50(0));
    R = sqrt( pow(P50(0),2) + pow(P50(1),2) );
    psi = acos( d[3] / R );
    th1[0] = phi + psi + M_PI_2;
    th1[1] = phi - psi + M_PI_2;

    //theta5
    th5[0] =  acos( ( pos(0)*sin(th1[0]) - pos(1)*cos(th1[0]) - d[3] ) / d[5] );
    th5[1] = -acos( ( pos(0)*sin(th1[0]) - pos(1)*cos(th1[0]) - d[3] ) / d[5] );
    th5[2] =  acos( ( pos(0)*sin(th1[1]) - pos(1)*cos(th1[1]) - d[3] ) / d[5] );
    th5[3] = -acos( ( pos(0)*sin(th1[1]) - pos(1)*cos(th1[1]) - d[3] ) / d[5] );

    //theta6
    T06 = T60.inverse();
    X06 << T06(0,0), T06(1,0), T06(2,0);
    Y06 << T06(0,1), T06(1,1), T06(2,1);
    th6[0] = atan2( ( (-X06(1)*sin(th1[0]) + Y06[1]*cos(th1[0])) / sin(th5[0]) ) , ( (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[0]) ) );
    th6[1] = atan2( ( (-X06(1)*sin(th1[0]) + Y06[1]*cos(th1[0])) / sin(th5[1]) ) , ( (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[1]) ) );
    th6[2] = atan2( ( (-X06(1)*sin(th1[1]) + Y06[1]*cos(th1[1])) / sin(th5[2]) ) , ( (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[2]) ) );
    th6[3] = atan2( ( (-X06(1)*sin(th1[1]) + Y06[1]*cos(th1[1])) / sin(th5[3]) ) , ( (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[3]) ) );

    //theta3
    //T41 = T61*(T54*T65).inverse()
    T(T54, th5[0], 4);
    T(T65, th6[0], 5);
    T(T10, th1[0], 0);
    T61 = T10.inverse() * T60;
    T41 = T61 * T54.inverse() * T65.inverse();
    tmp << 0, -d[3], 0, 1;
    P31 = T41 * tmp;
    n = P31.norm();
    
}

#endif
