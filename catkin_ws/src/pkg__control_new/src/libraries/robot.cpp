#include "robot.h"

ur5::Robot::Robot() {}

ur5::Position ur5::Robot::getPosition() {
    return this->position;
}

ur5::Orientation ur5::Robot::getOrientation() {
    return this->orientation;
}

ur5::JointAngles ur5::Robot::getJointAngles() {
    return this->jointAngles;
}

ur5::Pose tMatrix(float q, int index) {
    ur5::Pose m;

    switch(index) {
        case 0 :
        m << cos(q), -sin(q), 0, 0,
              sin(q), cos(q), 0, 0,
              0, 0, 1, ur5::d1,
              0, 0, 0, 1;
        break;

        case 1:
        m << cos(q), -sin(q), 0, 0,
            0, 0, -1, 0,
            sin(q), cos(q), 0, 0,
            0, 0, 0, 1;
        break;

        case 2:
        m << cos(q), -sin(q), 0, ur5::a2,
            sin(q), cos(q), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        break;

        case 3:
        m << cos(q), -sin(q), 0, ur5::a3,
            sin(q), cos(q), 0, 0,
            0, 0, 1, ur5::d4,
            0, 0, 0, 1;
        break;

        case 4:
        m << cos(q), -sin(q), 0, 0,
            0, 0, -1, -ur5::d5,
            sin(q), cos(q), 0, 0,
            0, 0, 0, 1;
        break;

        case 5:
        m << cos(q), -sin(q), 0, 0,
            0, 0, 1, ur5::d6,
            -sin(q), -cos(q), 0, 0,
            0, 0, 0, 1;
        break;
    }

    return m;
}

ur5::Pose ur5::Robot::computeDirect(const ur5::JointAngles &ja) {
    ur5::Pose TM;
    ur5::Pose T[ur5::noJoints];

    for(int i=0; i<ur5::noJoints; i++) {
        T[i] = tMatrix(ja(i), i);
    }

    TM = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

    this->orientation = TM.block<3,3>(0,0);
    this->position = TM.block<3,1>(0,3);

    return TM;
}

ur5::Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
    Eigen::Matrix4d T65, T54, T10;
    Eigen::MatrixXd T61, T41;
    Eigen::Vector4d tmp, P31;

    T65 = tMatrix(th6, 5);
    T54 = tMatrix(th5, 4);
    T10 = tMatrix(th1, 0);
    
    T61 = T10.inverse() * T60;
    T41 = T61 * T54.inverse() * T65.inverse();
    tmp << 0, -ur5::d[3], 0, 1;
    P31 = T41 * tmp;

    return P31;   
}

ur5::JointAngles ur5::Robot::computeInverse(const ur5::Pose &p) {
    ur5::JointAngles ja;
    Eigen::MatrixXd T60, T06;
    Eigen::Matrix3d ori;
    ur5::Pose T65, T54, T43, T32, T21, T10;
    Eigen::Vector3d X06, Y06, pos;
    Vector4d cmp, tmp, P50, P31[4];
    double phi, psi, R, n;
    double th1[2], th2[8], th3[8], th4[8], th5[4], th6[4];

    orientation = p.block<3,3>(0,0);
    position = p.block<3,1>(0,3);

    cmp << 0,0,0,1;
    T60.resize(3,3);
    T60 << orientation;
    T60.conservativeResize(Eigen::NoChange, T60.cols()+1);
    T60.col(T60.cols()-1) = position;
    T60.conservativeResize(T60.rows()+1, Eigen::NoChange);
    T60.row(T60.rows()-1) = cmp;
    T60.conservativeResize(4,4);

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
    T06.resize(4,4);
    T06 = T60.inverse();
    X06 << T06(0,0), T06(1,0), T06(2,0);
    Y06 << T06(0,1), T06(1,1), T06(2,1);
    th6[0] = atan2( (-X06(1)*sin(th1[0]) + Y06(1)*cos(th1[0])) / sin(th5[0]) , (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[0]) );
    th6[1] = atan2( (-X06(1)*sin(th1[0]) + Y06(1)*cos(th1[0])) / sin(th5[1]) , (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[1]) );
    th6[2] = atan2( (-X06(1)*sin(th1[1]) + Y06(1)*cos(th1[1])) / sin(th5[2]) , (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[2]) );
    th6[3] = atan2( (-X06(1)*sin(th1[1]) + Y06(1)*cos(th1[1])) / sin(th5[3]) , (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[3]) );

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

    //th4
    T65 = tMatrix(th6[0], 5);
    T54 = tMatrix(th5[0], 4);
    T32 = tMatrix(th3[0], 2);
    T21 = tMatrix(th2[0], 1);
    T10 = tMatrix(th1[0], 0);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v0(T43(0,0), T43(1,0), T43(2,0));
    th4[0] = atan2(v0(1), v0(0));

    T65 = tMatrix(th6[1], 5);
    T54 = tMatrix(th5[1], 4);
    T32 = tMatrix(th3[1], 2);
    T21 = tMatrix(th2[1], 1);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v1(T43(0,0), T43(1,0), T43(2,0));
    th4[1] = atan2(v1(1), v1(0));

    T65 = tMatrix(th6[2], 5);
    T54 = tMatrix(th5[2], 4);
    T32 = tMatrix(th3[2], 2);
    T21 = tMatrix(th2[2], 1);
    T10 = tMatrix(th1[1], 0);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v2(T43(0,0), T43(1,0), T43(2,0));
    th4[2] = atan2(v2(1), v2(0));

    T65 = tMatrix(th6[3], 5);
    T54 = tMatrix(th5[3], 4);
    T32 = tMatrix(th3[3], 2);
    T21 = tMatrix(th2[3], 1);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v3(T43(0,0), T43(1,0), T43(2,0));
    th4[3] = atan2(v3(1), v3(0));

    T65 = tMatrix(th6[0], 5);
    T54 = tMatrix(th5[0], 4);
    T32 = tMatrix(th3[4], 2);
    T21 = tMatrix(th2[4], 1);
    T10 = tMatrix(th1[0], 0);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v4(T43(0,0), T43(1,0), T43(2,0));
    th4[4] = atan2(v4(1), v4(0));

    T65 = tMatrix(th6[1], 5);
    T54 = tMatrix(th5[1], 4);
    T32 = tMatrix(th3[5], 2);
    T21 = tMatrix(th2[5], 1);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v5(T43(0,0), T43(1,0), T43(2,0));
    th4[5] = atan2(v5(1), v5(0));

    T65 = tMatrix(th6[2], 5);
    T54 = tMatrix(th5[2], 4);
    T32 = tMatrix(th3[6], 2);
    T21 = tMatrix(th2[6], 1);
    T10 = tMatrix(th1[1], 0);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v6(T43(0,0), T43(1,0), T43(2,0));
    th4[6] = atan2(v6(1), v6(0));

    T65 = tMatrix(th6[3], 5);
    T54 = tMatrix(th5[3], 4);
    T32 = tMatrix(th3[7], 2);
    T21 = tMatrix(th2[7], 1);
    T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Eigen::Vector3d v7(T43(0,0), T43(1,0), T43(2,0));
    th4[7] = atan2(v7(1), v7(0));

    ja << th1[1], th2[0], th3[0], th4[0], th5[3], th6[3];

    return ja;
}