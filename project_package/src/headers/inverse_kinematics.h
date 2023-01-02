#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include "direct_kinematics.h"

#define _USE_MATH_DEFINES

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Vector3d BlockPosition;

class Destination{
private:
    Eigen::Vector3d position;
    JointConfiguration ja;
public:
    Destination();
    Destination(const BlockPosition& pos);
    Eigen::Vector3d get_position() const;
    JointConfiguration get_joint_angles() const;
    void compute_inverse(const EndEffector& ee);
};

Destination::Destination(){
    position << Eigen::Vector3d::Zero();
}

Destination::Destination(const BlockPosition& bp){
    position = bp;
}

Eigen::Vector3d Destination::get_position() const{
    return this->position;
}

JointConfiguration Destination::get_joint_angles() const{
    return this->ja;
}

Eigen::Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
    Eigen::Matrix4d T65, T54, T10;
    Eigen::MatrixXd T61, T41;
    Eigen::Vector4d tmp, P31;

    T(T65, th6, 5);
    T(T54, th5, 4);
    T(T10, th1, 0);
    
    T61 = T10.inverse() * T60;
    T41 = T61 * T54.inverse() * T65.inverse();
    tmp << 0, -d[3], 0, 1;
    P31 = T41 * tmp;

    return P31;   
}

void Destination::compute_inverse(const EndEffector& ee){
    Eigen::MatrixXd T60, T06, ori;
    Matrix4d T43;
    Eigen::Vector3d X06, Y06, pos;
    Vector4d cmp, tmp, P50, P31[4];
    double phi, psi, R, n;
    double th1[2], th2[8], th3[8], th4[8], th5[4], th6[4];

    pos = this->position;
    ori = ee.get_orientation();     //what is the desired orientation? In the meantime, I put the current orientation

    //T60(th1,th2,th3,th4,th5,th6) = T10(th1)*T21(th2)*T32(th3)*T43(th4)*T54(th5)*T65(th6)
    cmp << 0,0,0,1;
    T60.resize(3,3);
    T60 << ee.get_orientation();
    T60.conservativeResize(Eigen::NoChange, T60.cols()+1);
    T60.col(T60.cols()-1) = pos;
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
    P31[0] = P(th1[0], th5[0], th6[0], T60);
    P31[1] = P(th1[0], th5[1], th6[1], T60);
    P31[2] = P(th1[1], th5[2], th6[2], T60);
    P31[3] = P(th1[1], th5[3], th6[3], T60);
    n = P31[0].norm();
    th3[0] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
    n = P31[1].norm();
    th3[1] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
    n = P31[2].norm();
    th3[2] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
    n = P31[3].norm();
    th3[3] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
    th3[4] = -th3[0];
    th3[5] = -th3[1];
    th3[6] = -th3[2];
    th3[7] = -th3[3];

    //theta2
    th2[0] = -atan2(P31[0](1),-P31[0](0)) + asin(cn[2]*sin(th3[0]) / P31[0].norm());
    th2[1] = -atan2(P31[1](1),-P31[1](0)) + asin(cn[2]*sin(th3[1]) / P31[1].norm());
    th2[2] = -atan2(P31[2](1),-P31[2](0)) + asin(cn[2]*sin(th3[2]) / P31[2].norm());
    th2[3] = -atan2(P31[3](1),-P31[3](0)) + asin(cn[2]*sin(th3[3]) / P31[3].norm());
    th2[4] = -atan2(P31[0](1),-P31[0](0)) + asin(cn[2]*sin(th3[4]) / P31[0].norm());
    th2[5] = -atan2(P31[1](1),-P31[1](0)) + asin(cn[2]*sin(th3[5]) / P31[1].norm());
    th2[6] = -atan2(P31[2](1),-P31[2](0)) + asin(cn[2]*sin(th3[6]) / P31[2].norm());
    th2[7] = -atan2(P31[3](1),-P31[3](0)) + asin(cn[2]*sin(th3[7]) / P31[3].norm());

    //theta4
    T(T43, th4[0], 3);
    th4[0] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[1], 3);
    th4[1] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[2], 3);
    th4[2] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[3], 3);
    th4[3] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[4], 3);
    th4[4] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[5], 3);
    th4[5] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[6], 3);
    th4[6] = atan2(T43(1,0), T43(0,0));
    T(T43, th4[7], 3);
    th4[7] = atan2(T43(1,0), T43(0,0));

    this->ja << th1[0], th2[0], th3[0], th4[0], th5[0], th6[0];
}

#endif
