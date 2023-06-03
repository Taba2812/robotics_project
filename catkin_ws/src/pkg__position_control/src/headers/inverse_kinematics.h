#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include "direct_kinematics.h"
#include "std_msgs/Float32MultiArray.h"


class Destination{
private:
    Eigen::Vector3d position;
    JointConfiguration jc;
public:
    Destination();
    Destination(const BlockPosition& pos);
    bool isWithinJointLimits(const Eigen::VectorXd& joints);
    Eigen::VectorXd applyJointLimits(const Eigen::VectorXd& joints);
    Eigen::Vector3d getPosition() const;
    JointConfiguration getJointAngles() const;
    std_msgs::Float32MultiArray getMessage() const;
    void computeInverse(const EndEffector& ee);
    std_msgs::Float32MultiArray getDestination();
    std_msgs::Float64MultiArray getMessage();
    void setPosition(Eigen::Vector3d p);
};

Destination::Destination(){
    position << Eigen::Vector3d::Zero();
}

Destination::Destination(const BlockPosition& bp){
    position = bp;
}

Eigen::Vector3d Destination::getPosition() const{
    return this->position;
}

JointConfiguration Destination::getJointAngles() const{
    return this->jc;
}

Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
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

void Destination::computeInverse(const EndEffector &ee) {
    Eigen::VectorXd th(6);

    // std::cout << "IK Position: " << ee.getPosition() << "\n\n";
    // std::cout << "Orientation: " << ee.getOrientation() << "\n\n";

    //position and orientation
    const Eigen::Vector3d &translation = ee.getPosition();
    const Eigen::Quaterniond orientation(ee.getOrientation());
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();

    //wrist center position
    const Eigen::Vector3d wristCenter = translation - D6 * rotation.col(2);

    //theta1
    th(0) = std::atan2(wristCenter(1), wristCenter(0));

    //theta3
    const double r = std::sqrt(std::pow(wristCenter(0), 2) + std::pow(wristCenter(1), 2));
    const double s = wristCenter(2) - D1;
    const double A = (std::pow(r, 2) + std::pow(s, 2) - std::pow(A2, 2) - std::pow(A3, 2)) / (2 * A2 * A3);

    th(2) = std::atan2(-std::sqrt(1 - std::pow(A, 2)), A);

    std::cout << "TH3: " << th(2) << "\n\n";

    //theta2
    const double B = A3 * std::sin(th(2)) / std::sqrt(std::pow(r, 2) + std::pow(s, 2));
    th(1) = std::atan2(s, r) - std::atan2(B, std::sqrt(1 - std::pow(B, 2)));

    //rotation matrix from base to end-effector
    Eigen::AngleAxisd x = Eigen::AngleAxisd(th(2), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd y = Eigen::AngleAxisd(th(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd z = Eigen::AngleAxisd(th(0), Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond Q0_3 = z * y * x;
    const Eigen::Matrix3d R0_3 = Q0_3.toRotationMatrix();
    const Eigen::Matrix3d R3_6 = R0_3.transpose() * rotation;

    //theta4
    th(3) = std::atan2(R3_6(2, 1), R3_6(2, 2));

    //theta5
    th(4) = std::atan2(-R3_6(2, 0), std::sqrt(R3_6(2, 1) * R3_6(2, 1) + R3_6(2, 2) * R3_6(2, 2)));

    //theta6
    th(5) = std::atan2(R3_6(1, 0), R3_6(0, 0));

    //apply joint limits
    th = applyJointLimits(th);

    this->jc = th;
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

// void Destination::computeInverse(const EndEffector  &ee){
//     Eigen::MatrixXd T60, T06;
//     Eigen::Matrix3d ori;
//     Matrix4d T65, T54, T43, T32, T21, T10;
//     Eigen::Vector3d X06, Y06, pos;
//     Vector4d cmp, tmp, P50, P31[4];
//     double phi, psi, R, n;
//     double th1[2], th2[8], th3[8], th4[8], th5[4], th6[4];

//     pos = this->position;
//     ori = ee.getOrientation();     //doesn't really matter since we use a dynamic link for the gripper

//     //T60(th1,th2,th3,th4,th5,th6) = T10(th1)*T21(th2)*T32(th3)*T43(th4)*T54(th5)*T65(th6)
//     cmp << 0,0,0,1;
//     T60.resize(3,3);
//     T60 << ee.getOrientation();
//     T60.conservativeResize(Eigen::NoChange, T60.cols()+1);
//     T60.col(T60.cols()-1) = pos;
//     T60.conservativeResize(T60.rows()+1, Eigen::NoChange);
//     T60.row(T60.rows()-1) = cmp;
//     T60.conservativeResize(4,4);

//     //theta1
//     tmp << 0,0,-d[5],1;
//     P50 = T60 * tmp;
//     phi = atan2(P50(1), P50(0));
//     R = sqrt( pow(P50(0),2) + pow(P50(1),2) );
//     psi = acos( d[3] / R );
//     th1[0] = phi + psi + M_PI_2;
//     th1[1] = phi - psi + M_PI_2;

//     //theta5
//     th5[0] =  acos( ( pos(0)*sin(th1[0]) - pos(1)*cos(th1[0]) - d[3] ) / d[5] );
//     th5[1] = -acos( ( pos(0)*sin(th1[0]) - pos(1)*cos(th1[0]) - d[3] ) / d[5] );
//     th5[2] =  acos( ( pos(0)*sin(th1[1]) - pos(1)*cos(th1[1]) - d[3] ) / d[5] );
//     th5[3] = -acos( ( pos(0)*sin(th1[1]) - pos(1)*cos(th1[1]) - d[3] ) / d[5] );

//     //theta6
//     T06.resize(4,4);
//     T06 = T60.inverse();
//     X06 << T06(0,0), T06(1,0), T06(2,0);
//     Y06 << T06(0,1), T06(1,1), T06(2,1);
//     th6[0] = atan2( ( (-X06(1)*sin(th1[0]) + Y06[1]*cos(th1[0])) / sin(th5[0]) ) , ( (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[0]) ) );
//     th6[1] = atan2( ( (-X06(1)*sin(th1[0]) + Y06[1]*cos(th1[0])) / sin(th5[1]) ) , ( (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[1]) ) );
//     th6[2] = atan2( ( (-X06(1)*sin(th1[1]) + Y06[1]*cos(th1[1])) / sin(th5[2]) ) , ( (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[2]) ) );
//     th6[3] = atan2( ( (-X06(1)*sin(th1[1]) + Y06[1]*cos(th1[1])) / sin(th5[3]) ) , ( (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[3]) ) );

//     //theta3
//     P31[0] = P(th1[0], th5[0], th6[0], T60);
//     P31[1] = P(th1[0], th5[1], th6[1], T60);
//     P31[2] = P(th1[1], th5[2], th6[2], T60);
//     P31[3] = P(th1[1], th5[3], th6[3], T60);
//     n = P31[0].norm();
//     th3[0] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
//     n = P31[1].norm();
//     th3[1] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
//     n = P31[2].norm();
//     th3[2] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
//     n = P31[3].norm();
//     th3[3] = acos((pow(n,2)-pow(cn[1],2)-pow(cn[2],2)) / 2*cn[1]*cn[2]);
//     th3[4] = -th3[0];
//     th3[5] = -th3[1];
//     th3[6] = -th3[2];
//     th3[7] = -th3[3];

//     //theta2
//     th2[0] = -atan2(P31[0](1),-P31[0](0)) + asin(cn[2]*sin(th3[0]) / P31[0].norm());
//     th2[1] = -atan2(P31[1](1),-P31[1](0)) + asin(cn[2]*sin(th3[1]) / P31[1].norm());
//     th2[2] = -atan2(P31[2](1),-P31[2](0)) + asin(cn[2]*sin(th3[2]) / P31[2].norm());
//     th2[3] = -atan2(P31[3](1),-P31[3](0)) + asin(cn[2]*sin(th3[3]) / P31[3].norm());
//     th2[4] = -atan2(P31[0](1),-P31[0](0)) + asin(cn[2]*sin(th3[4]) / P31[0].norm());
//     th2[5] = -atan2(P31[1](1),-P31[1](0)) + asin(cn[2]*sin(th3[5]) / P31[1].norm());
//     th2[6] = -atan2(P31[2](1),-P31[2](0)) + asin(cn[2]*sin(th3[6]) / P31[2].norm());
//     th2[7] = -atan2(P31[3](1),-P31[3](0)) + asin(cn[2]*sin(th3[7]) / P31[3].norm());

//     //th4
//     T(T65, th6[0], 5);
//     T(T54, th5[0], 4);
//     T(T32, th3[0], 2);
//     T(T21, th2[0], 1);
//     T(T10, th1[0], 0);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[0] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[1], 5);
//     T(T54, th5[1], 4);
//     T(T32, th3[1], 2);
//     T(T21, th2[1], 1);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[1] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[2], 5);
//     T(T54, th5[2], 4);
//     T(T32, th3[2], 2);
//     T(T21, th2[2], 1);
//     T(T10, th1[1], 0);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[2] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[3], 5);
//     T(T54, th5[3], 4);
//     T(T32, th3[3], 2);
//     T(T21, th2[3], 1);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[3] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[0], 5);
//     T(T54, th5[0], 4);
//     T(T32, th3[4], 2);
//     T(T21, th2[4], 1);
//     T(T10, th1[0], 0);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[4] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[1], 5);
//     T(T54, th5[1], 4);
//     T(T32, th3[5], 2);
//     T(T21, th2[5], 1);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[5] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[2], 5);
//     T(T54, th5[2], 4);
//     T(T32, th3[6], 2);
//     T(T21, th2[6], 1);
//     T(T10, th1[1], 0);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[6] = atan2(T43(1,0), T43(0,0));

//     T(T65, th6[3], 5);
//     T(T54, th5[3], 4);
//     T(T32, th3[7], 2);
//     T(T21, th2[7], 1);
//     T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
//     th4[7] = atan2(T43(1,0), T43(0,0));

//     this->jc << th1[0], th2[2], th3[2], th4[2], th5[0], th6[0];
// }

std_msgs::Float32MultiArray Destination::getDestination(){
    std_msgs::Float32MultiArray destination;
    destination.data[0] = this->getPosition()[0];
    destination.data[1] = this->getPosition()[1];
    destination.data[2] = this->getPosition()[2];

    return destination;
}

std_msgs::Float64MultiArray Destination::getMessage(){
    std_msgs::Float64MultiArray msg;

    for(int i=0; i<JOINTS; i++){
        msg.data[i] = jc(i);
    }

    return msg;
}

void Destination::setPosition(Eigen::Vector3d p){
    position = p;
}

#endif