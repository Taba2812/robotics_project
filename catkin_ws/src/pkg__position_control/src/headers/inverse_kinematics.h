#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include "direct_kinematics.h"
#include "std_msgs/Float32MultiArray.h"

typedef Eigen::Matrix<double, 4, 1> Vector4d;

class Destination{
private:
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
    JointAngles jointAngles;
public:
    Destination();
    Destination(const Position &bp, const Orientation &bo);

    Eigen::Vector3d getPosition() const;
    Eigen::Matrix3d getOrientation() const;
    JointAngles getJointAngles() const;

    void setPosition(const Position bp);
    void setOrientation(const Orientation o);
    void setJointAngles(const JointAngles);
    
    bool isWithinJointLimits(const Eigen::VectorXd& joints);
    Eigen::VectorXd applyJointLimits(const Eigen::VectorXd& joints);
    Eigen::Vector3d transformCoordinates(const Eigen::Vector3d &pointA, const Eigen::Vector3d &originA, const Eigen::Vector3d &originB, double roll, double pitch, double yaw);

    JointAngles computeInverse(const EndEffector& ee);
};

Destination::Destination(){
    position << Eigen::Vector3d::Zero();
}

Destination::Destination(const Position& bp, const Orientation &bo){
    position = bp;
    orientation = bo;
}

Eigen::Vector3d Destination::getPosition() const{
    return this->position;
}

Eigen::Matrix3d Destination::getOrientation() const {
    return this->orientation;
}

JointAngles Destination::getJointAngles() const{
    return this->jointAngles;
}

void Destination::setPosition(const Position bp){
    position = bp;
}

void Destination::setOrientation(const Orientation bo){
    orientation = bo;
}

void Destination::setJointAngles(const JointAngles ja) {
    jointAngles = ja;
}

Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
    Eigen::Matrix4d T65, T54, T10;
    Eigen::MatrixXd T61, T41;
    Eigen::Vector4d tmp, P31;

    T65 = tMatrix(th6, 5);
    T54 = tMatrix(th5, 4);
    T10 = tMatrix(th1, 0);
    
    T61 = T10.inverse() * T60;
    T41 = T61 * T54.inverse() * T65.inverse();
    tmp << 0, -d[3], 0, 1;
    P31 = T41 * tmp;

    return P31;   
}

Eigen::Matrix3d rotationMatrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d rm;
    rm << cos(yaw) * cos(pitch),
          cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),
          cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),

          sin(yaw) * cos(pitch),
          sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
          sin(yaw) * sin(pitch) * sin(roll) - cos(yaw) * sin(roll),

          -sin(pitch),
          cos(pitch) * sin(roll),
          cos(pitch) * cos(roll);

    return rm;
}

Eigen::Vector3d Destination::transformCoordinates(const Eigen::Vector3d &pointA, const Eigen::Vector3d &originA, const Eigen::Vector3d &originB, double roll, double pitch, double yaw) {
    Eigen::Matrix3d transformation = rotationMatrix(roll, pitch, yaw);
    
    Eigen::Vector3d p = pointA - originA;
    Eigen::Vector3d q = transformation * p;
    Eigen::Vector3d pointB = q + originB;

    this -> position = pointB;

    return pointB;
}

JointAngles Destination::computeInverse(const EndEffector  &ee){
    JointAngles ja;
    Eigen::MatrixXd T60, T06;
    Eigen::Matrix3d ori;
    Matrix4d T65, T54, T43, T32, T21, T10;
    Eigen::Vector3d X06, Y06, pos;
    Vector4d cmp, tmp, P50, P31[4];
    double phi, psi, R, n;
    double th1[2], th2[8], th3[8], th4[8], th5[4], th6[4];

    pos = ee.getPosition();
    ori = ee.getOrientation();

    cmp << 0,0,0,1;
    T60.resize(3,3);
    T60 << ee.getOrientation();
    T60.conservativeResize(Eigen::NoChange, T60.cols()+1);
    T60.col(T60.cols()-1) = pos;
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
    this -> jointAngles = ja;

    return ja;
}

bool Destination::isWithinJointLimits(const Eigen::VectorXd &joints){
  // Check joint limits for each joint
  if (joints(0) < joint1_min || joints(0) > joint1_max) return false;
  if (joints(1) < joint2_min || joints(1) > joint2_max) return false;
  if (joints(2) < joint3_min || joints(2) > joint3_max) return false;
  if (joints(3) < joint4_min || joints(3) > joint4_max) return false;
  if (joints(4) < joint5_min || joints(4) > joint5_max) return false;
  if (joints(5) < joint6_min || joints(5) > joint6_max) return false;

  return true;
}

Eigen::VectorXd Destination::applyJointLimits(const Eigen::VectorXd& joints) {
  Eigen::VectorXd limitedJoints(joints);

  // Apply joint limits for each joint
  limitedJoints(0) = std::max(joint1_min, std::min(joint1_max, limitedJoints(0)));
  limitedJoints(1) = std::max(joint2_min, std::min(joint2_max, limitedJoints(1)));
  limitedJoints(2) = std::max(joint3_min, std::min(joint3_max, limitedJoints(2)));
  limitedJoints(3) = std::max(joint4_min, std::min(joint4_max, limitedJoints(3)));
  limitedJoints(4) = std::max(joint5_min, std::min(joint5_max, limitedJoints(4)));
  limitedJoints(5) = std::max(joint6_min, std::min(joint6_max, limitedJoints(5)));

  return limitedJoints;
}

#pragma region comment
// JointAngles Destination::computeInverse(const EndEffector &ee) {
//     JointAngles th(JOINTS);

//     // std::cout << "IK Position: " << ee.getPosition() << "\n\n";
//     // std::cout << "Orientation: " << ee.getOrientation() << "\n\n";

//     //position and orientation
//     const Eigen::Vector3d &translation = ee.getPosition();
//     const Eigen::Quaterniond orientation(ee.getOrientation());
//     const Eigen::Matrix3d rotation = orientation.toRotationMatrix();

//     //wrist center position
//     const Eigen::Vector3d wristCenter = translation - D6 * rotation.col(2);

//     //theta1
//     th(0) = std::atan2(wristCenter(1), wristCenter(0));

//     //theta3
//     const double r = std::sqrt(std::pow(wristCenter(0), 2) + std::pow(wristCenter(1), 2));
//     const double s = wristCenter(2) - D1;
//     const double A = (std::pow(r, 2) + std::pow(s, 2) - std::pow(A2, 2) - std::pow(A3, 2)) / (2 * A2 * A3);

//     th(2) = std::atan2(-std::sqrt(1 - std::pow(A, 2)), A);

//     //theta2
//     const double B = A3 * std::sin(th(2)) / std::sqrt(std::pow(r, 2) + std::pow(s, 2));
//     th(1) = std::atan2(s, r) - std::atan2(B, std::sqrt(1 - std::pow(B, 2)));

//     //rotation matrix from base to end-effector
//     Eigen::AngleAxisd x = Eigen::AngleAxisd(th(2), Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd y = Eigen::AngleAxisd(th(1), Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd z = Eigen::AngleAxisd(th(0), Eigen::Vector3d::UnitZ());
//     const Eigen::Quaterniond Q0_3 = z * y * x;
//     const Eigen::Matrix3d R0_3 = Q0_3.toRotationMatrix();
//     const Eigen::Matrix3d R3_6 = R0_3.transpose() * rotation;

//     //theta4
//     th(3) = std::atan2(R3_6(2, 1), R3_6(2, 2));

//     //theta5
//     th(4) = std::atan2(-R3_6(2, 0), std::sqrt(R3_6(2, 1) * R3_6(2, 1) + R3_6(2, 2) * R3_6(2, 2)));

//     //theta6
//     th(5) = std::atan2(R3_6(1, 0), R3_6(0, 0));

//     //apply joint limits
//     th = applyJointLimits(th);

//     this -> jointAngles = th;
//     return th;
// }
#pragma endregion comment

#endif