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

// ur5::Pose tMatrix(float q, int index) {
//     ur5::Pose m;

//     switch(index) {
//         case 0 :
//         m << cos(q), -sin(q), 0, 0,
//              sin(q), cos(q), 0, 0,
//              0, 0, 1, ur5::d1,
//              0, 0, 0, 1;
//         break;

//         case 1:
//         m << cos(q), -sin(q), 0, ur5::a2 * cos(q),
//              sin(q), cos(q), 0, ur5::a2 * sin(q),
//              0, 0, 1, 0,
//              0, 0, 0, 1;
//         break;

//         case 2:
//         m << cos(q), -sin(q), 0, ur5::a3 * cos(q),
//              sin(q), cos(q), 0, ur5::a3 * sin(q),
//              0, 0, 1, 0,
//              0, 0, 0, 1;
//         break;

//         case 3:
//         m << cos(q), -sin(q), 0, 0,
//              sin(q), cos(q), 0, 0,
//              0, 0, 1, ur5::d4,
//              0, 0, 0, 1;
//         break;

//         case 4:
//         m << cos(q), -sin(q), 0, 0,
//              sin(q), cos(q), 0, 0,
//              0, 0, 1, ur5::d5,
//              0, 0, 0, 1;
//         break;

//         case 5:
//         m << cos(q), -sin(q), 0, 0,
//              sin(q), cos(q), 0, 0,
//              0, 0, 1, ur5::d6,
//              0, 0, 0, 1;
//         break;
//     }

//     return m;
// }

ur5::Pose ur5::Robot::computeDirect(const ur5::JointAngles &ja) {
    // ur5::Pose TM;
    // ur5::Pose T[ur5::noJoints];

    // for(int i=0; i<ur5::noJoints; i++) {
    //     T[i] = tMatrix(ja(i), i);
    // }


    // TM = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

    // this->orientation = TM.block<3,3>(0,0);
    // this->position = TM.block<3,1>(0,3);

    // return TM;


    // joint 1
    ur5::JointAngles j1_1;
    j1_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j1_2;
    j1_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    
    ur5::Pose j1_matrix;
    j1_matrix << cos(ja(0)), -sin(ja(0)), 0, 0, sin(ja(0)), cos(ja(0)), 0, 0, 0, 0, 1, j1_2(0), 0, 0, 0, 1;

    //joint 2
    ur5::JointAngles j2_1;
    j2_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j2_2;
    j2_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    ur5::Pose j2_matrix;
    j2_matrix << cos(ja(1)), -sin(ja(1)), 0, 0, 0, 0, -1, 0, sin(ja(1)), cos(ja(1)), 0, 0, 0, 0, 0, 1;

    // joint 3
    ur5::JointAngles j3_1;
    j3_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j3_2;
    j3_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    ur5::Pose j3_matrix;
    j3_matrix << cos(ja(2)), -sin(ja(2)), 0, j3_1(1), sin(ja(2)), cos(ja(2)), 0, 0, 0, 0, 1, j3_2(2), 0, 0, 0, 1;

    // joint 4
    ur5::JointAngles j4_1;
    j4_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j4_2;
    j4_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    ur5::Pose j4_matrix;
    j4_matrix << cos(ja(3)), -sin(ja(3)), 0, j4_1(2), sin(ja(3)), cos(ja(3)), 0, 0, 0, 0, 1, j4_2(3), 0, 0, 0, 1;

    // joint 5
    ur5::JointAngles j5_1;
    j5_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j5_2;
    j5_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    ur5::Pose j5_matrix;
    j5_matrix << cos(ja(4)), -sin(ja(4)), 0, 0, 0, 0, -1, -j5_2(4), sin(ja(4)), cos(ja(4)), 0, 0, 0, 0, 0, 1;

    // joint 6
    ur5::JointAngles j6_1;
    j6_1 << 0, -0.425, -0.3922, 0, 0, 0;

    ur5::JointAngles j6_2;
    j6_2 << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    ur5::Pose j6_matrix;
    j6_matrix << cos(ja(5)), -sin(ja(5)), 0, 0, 0, 0, 1, j6_2(5), -sin(ja(5)), -cos(ja(5)), 0, 0, 0, 0, 0, 1;

    ur5::Pose retMatrix = j1_matrix * j2_matrix * j3_matrix * j4_matrix * j5_matrix * j6_matrix;

    return retMatrix;
}

// ur5::Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
//     Eigen::Matrix4d T65, T54, T10;
//     Eigen::MatrixXd T61, T41;
//     Eigen::Vector4d tmp, P31;

//     T65 = tMatrix(th6, 5);
//     T54 = tMatrix(th5, 4);
//     T10 = tMatrix(th1, 0);
    
//     T61 = T10.inverse() * T60;
//     T41 = T61 * T54.inverse() * T65.inverse();
//     tmp << 0, -ur5::d[3], 0, 1;
//     P31 = T41 * tmp;

//     return P31;   
// }

ur5::JointAngles ur5::Robot::computeInverse(const ur5::Position &p, const ur5::Orientation &o) {
    ur5::JointAngles th(ur5::noJoints);

    // std::cout << "IK Position: " << ee.getPosition() << "\n\n";
    // std::cout << "Orientation: " << ee.getOrientation() << "\n\n";

    //position and orientation
    const Eigen::Vector3d &translation = p;
    const Eigen::Quaterniond orientation(o);
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();

    //wrist center position
    const Eigen::Vector3d wristCenter = translation - ur5::d6 * rotation.col(2);

    //theta1
    th(0) = std::atan2(wristCenter(1), wristCenter(0));

    //theta3
    const double r = std::sqrt(std::pow(wristCenter(0), 2) + std::pow(wristCenter(1), 2));
    const double s = wristCenter(2) - ur5::d1;
    const double A = (std::pow(r, 2) + std::pow(s, 2) - std::pow(ur5::a2, 2) - std::pow(ur5::a3, 2)) / (2 * ur5::a2 * ur5::a3);

    th(2) = std::atan2(-std::sqrt(1 - std::pow(A, 2)), A);

    //theta2
    const double B = ur5::a3 * std::sin(th(2)) / std::sqrt(std::pow(r, 2) + std::pow(s, 2));
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
    // th = applyJointLimits(th);

    return th;
}

std_msgs::Float32MultiArray ur5::Robot::motionPlanningMessage(const Position &initial, const Position &final, const int &noSteps) {
    std_msgs::Float32MultiArray motionMsg;
    motionMsg.data.resize(7);

    motionMsg.data.at(0) = initial(0);
    motionMsg.data.at(1) = initial(1);
    motionMsg.data.at(2) = initial(2);
    motionMsg.data.at(3) = noSteps;
    motionMsg.data.at(4) = final(0);
    motionMsg.data.at(5) = final(1);
    motionMsg.data.at(6) = final(2);

    return motionMsg;
}

ur5::Orientation ur5::Robot::computeOrientation(const ur5::EulerAngles &p) {
    ur5::Orientation orientation;

    const double roll = p(0);
    const double pitch = p(1);
    const double yaw = p(2);

    orientation << cos(yaw) * cos(pitch),
                   cos(yaw) * sin(roll) * sin(pitch) - cos(roll) * sin(yaw),
                   cos(yaw) * cos(roll) * sin(pitch) + sin(yaw) * sin(roll),

                   sin(yaw) * cos(pitch),
                   sin(yaw) * sin(roll) * sin(pitch) + cos(roll) * cos(yaw),
                   sin(yaw) * cos(roll) * sin(pitch) - sin(roll) * cos(yaw),

                   -sin(pitch),
                   cos(pitch) * sin(roll),
                   cos(pitch) * cos(roll);

    return orientation;

}