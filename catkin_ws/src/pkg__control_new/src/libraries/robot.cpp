#include "robot.h"

#pragma region Pose

ur5::Pose::Pose() {
    this->matrix = Eigen::Matrix<double, 4, 4>::Identity();
}

ur5::Pose::Pose(Eigen::Vector3d position, Eigen::Vector3d orientation) {
    Eigen::Matrix<double,3,3> rotation_matrix;
    Eigen::Vector4d filler(0, 0, 0, 1);

    Eigen::Matrix3d roll_matrix;
    roll_matrix << 1, 0,                    0,
                   0, cos(orientation(0)), -sin(orientation(0)),
                   0, sin(orientation(0)),  cos(orientation(0));

    Eigen::Matrix3d pitch_matrix;
    pitch_matrix <<  cos(orientation(1)), 0, sin(orientation(1)),
                     0,                   1, 0,
                    -sin(orientation(1)), 0, cos(orientation(1));

    Eigen::Matrix3d yaw_matrix;
    yaw_matrix << cos(orientation(2)), -sin(orientation(2)), 0,
                  sin(orientation(2)),  cos(orientation(2)), 0,
                  0,                    0,                   1;

    rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix;
    
    this->matrix.block<3,3>(0,0) = rotation_matrix;
    this->matrix.block<3,1>(0,3) = position;
    this->matrix.block<1,4>(3,0) = filler;
}

ur5::Pose::Pose(Eigen::Vector3d position, Eigen::Matrix3d orientation) {
    Eigen::Vector4d filler(0, 0, 0, 1);
    
    this->matrix.block<3,3>(0,0) = orientation;
    this->matrix.block<3,1>(0,3) = position;
    this->matrix.block<1,4>(3,0) = filler;
}

ur5::Pose::Pose(Eigen::Matrix4d m) {
    this->matrix = m;
}

#pragma endregion

ur5::Pose tMatrix(float theta, float alpha, float d, float a) {
    ur5::Pose m;

    m.matrix << cos(theta), -sin(theta), 0, a,
         sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
         sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
         0, 0, 0, 1;
        
    return m;
}

ur5::Pose ur5::Robot::computeDirect(const JointAngles ja) {
    ur5::Pose TM;
    ur5::Pose T[NO_JOINTS];

    for(int i=0; i<NO_JOINTS; i++) {
        T[i] = tMatrix(ja(i), this->coef.alpha[i] , this->coef.d[i], this->coef.cn[i]);
    }

    TM =  T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

    this->eeWorld = this->world_displacement * TM;
    this->ee = TM;

    return this->world_displacement * TM;
}

ur5::Pose ur5::Robot::computeDirect() {
    return this -> computeDirect(this->joints_current);
}


ur5::Robot::Robot() {
    JointLimits limits = {
        //  0        1        2       3        4        5
        { 2.9671, -0.1,    -2.617,  0.1,     2.094,  6.2832},
        {-2.9671, -1.8326, -0.1,   -3.1416, -2.094, -6.2832}
    };
    Ur5Coefficients coef = {
        //  0        1        2       3        4        5
        {0.0f,    M_PI_2,   0.0f,   0.0f,   M_PI_2,  -M_PI_2},
        {0.0f,    0.0f,    -0.425, -0.3922, 0.0f,       0.0f},
        {0.1625,  0.0f,     0.0f,   0.1333, 0.0997,   0.0996}
    };

    this->joints_limits = limits;
    this->coef = coef;
    this->joints_current << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    //In the simulation the robot is rotated 180° on the x axis
    this->world_displacement = Pose(Eigen::Vector3d::Zero(), Eigen::Vector3d(M_PI,0,0));
    this->eeWorld = this->computeDirect();
}

ur5::Pose ur5::Robot::translateEndEffector(Eigen::Vector3d tr) {
    ur5::Pose tmp(this->eeWorld);
    tmp.matrix(0,3) += tr(0);
    tmp.matrix(1,3) += tr(1);
    tmp.matrix(2,3) += tr(2);

    return tmp;
}

ur5::JointAngles ur5::Robot::computeInverse(const Pose pose) { 
    ur5::JointAngles ja;
    Eigen::MatrixXd T60, T06;
    Eigen::Matrix3d ori;
    ur5::Pose T65, T54, T43, T32, T21, T10;
    Eigen::Vector3d X06, Y06, pos;
    Vector4d cmp, tmp, P50, P31[4];
    double phi, psi, R, n;
    double th1[2], th2[8], th3[8], th4[8], th5[4], th6[4];

    T60 = pose.matrix;

    std::cout << "T60" << std::endl;
    std::cout << T60 << std::endl;

    //theta1
    tmp << 0, 0, -(this->coef.d[5]),1;
    P50 = T60 * tmp;
    phi = atan2(P50(1), P50(0));
    R = sqrt( pow(P50(0),2) + pow(P50(1),2) );
    psi = acos( this->coef.d[3] / R );
    th1[0] = phi + psi + M_PI_2;
    th1[1] = phi - psi + M_PI_2;

    std::cout << "\np50: " << P50 << "\nTH1: " << th1[0] << "\n" << th1[1] << "\n";

    Eigen::Vector3d position = pose.getPosition();

    //theta5
    th5[0] =  acos( ( position(0)*sin(th1[0]) - position(1)*cos(th1[0]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[1] = -acos( ( position(0)*sin(th1[0]) - position(1)*cos(th1[0]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[2] =  acos( ( position(0)*sin(th1[1]) - position(1)*cos(th1[1]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[3] = -acos( ( position(0)*sin(th1[1]) - position(1)*cos(th1[1]) - this->coef.d[3] ) / this->coef.d[5] );

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
    ur5::Pose T41m_1(tMatrix(th1[0], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]).matrix.inverse() * T60* tMatrix(th6[0], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]).matrix.inverse() * tMatrix(th5[0], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]).matrix.inverse());
    Eigen::Vector3d p41_1 = T41m_1.matrix.block<3,1>(0,3);
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    ur5::Pose T41m_2(tMatrix(th1[0], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]).matrix.inverse() * T60* tMatrix(th6[1], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]).matrix.inverse() * tMatrix(th5[1], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]).matrix.inverse());
    Eigen::Vector3d p41_2 = T41m_2.matrix.block<3,1>(0,3);
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    ur5::Pose T41m_3(tMatrix(th1[1], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]).matrix.inverse() * T60* tMatrix(th6[2], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]).matrix.inverse() * tMatrix(th5[2], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]).matrix.inverse());
    Eigen::Vector3d p41_3 = T41m_3.matrix.block<3,1>(0,3);
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    ur5::Pose T41m_4(tMatrix(th1[1], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]).matrix.inverse() * T60* tMatrix(th6[3], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]).matrix.inverse() * tMatrix(th5[3], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]).matrix.inverse());
    Eigen::Vector3d p41_4 = T41m_4.matrix.block<3,1>(0,3);
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    th3[0] = acos( (pow(p41xz_1, 2) - pow(this->coef.cn[2], 2) - pow(this->coef.cn[3], 2)) / (2 * this->coef.cn[2] * this->coef.cn[3]));
    th3[1] = acos( (pow(p41xz_2, 2) - pow(this->coef.cn[2], 2) - pow(this->coef.cn[3], 2)) / (2 * this->coef.cn[2] * this->coef.cn[3]));
    th3[2] = acos( (pow(p41xz_3, 2) - pow(this->coef.cn[2], 2) - pow(this->coef.cn[3], 2)) / (2 * this->coef.cn[2] * this->coef.cn[3]));
    th3[3] = acos( (pow(p41xz_4, 2) - pow(this->coef.cn[2], 2) - pow(this->coef.cn[3], 2)) / (2 * this->coef.cn[2] * this->coef.cn[3]));
    th3[4] = -th3[0];
    th3[5] = -th3[1];
    th3[6] = -th3[2];
    th3[7] = -th3[3];

    //theta2
    th2[0] = atan2(-p41_1(2), -p41_1(0)) - asin( (-(this->coef.cn[3]) * sin(th3[0])) / (p41xz_1) );
    th2[1] = atan2(-p41_2(2), -p41_2(0)) - asin( (-(this->coef.cn[3]) * sin(th3[1])) / (p41xz_2) );
    th2[2] = atan2(-p41_3(2), -p41_3(0)) - asin( (-(this->coef.cn[3]) * sin(th3[2])) / (p41xz_3) );
    th2[3] = atan2(-p41_4(2), -p41_4(0)) - asin( (-(this->coef.cn[3]) * sin(th3[3])) / (p41xz_4) );
    th2[4] = atan2(-p41_1(2), -p41_1(0)) - asin( ( (this->coef.cn[3]) * sin(th3[0])) / (p41xz_1) );
    th2[5] = atan2(-p41_2(2), -p41_2(0)) - asin( ( (this->coef.cn[3]) * sin(th3[1])) / (p41xz_2) );
    th2[6] = atan2(-p41_3(2), -p41_3(0)) - asin( ( (this->coef.cn[3]) * sin(th3[2])) / (p41xz_3) );
    th2[7] = atan2(-p41_4(2), -p41_4(0)) - asin( ( (this->coef.cn[3]) * sin(th3[3])) / (p41xz_4) );

    //th4
    T65 = tMatrix(th6[0], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[0], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[0], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[0], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T10 = tMatrix(th1[0], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v0(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[0] = atan2(v0(1), v0(0));

    T65 = tMatrix(th6[1], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[1], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[1], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[1], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v1(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[1] = atan2(v1(1), v1(0));

    T65 = tMatrix(th6[2], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);;
    T54 = tMatrix(th5[2], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[2], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[2], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T10 = tMatrix(th1[1], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v2(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[2] = atan2(v2(1), v2(0));

    T65 = tMatrix(th6[3], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[3], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[3], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[3], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v3(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[3] = atan2(v3(1), v3(0));

    T65 = tMatrix(th6[0], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[0], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[4], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[4], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T10 = tMatrix(th1[0], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v4(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[4] = atan2(v4(1), v4(0));

    T65 = tMatrix(th6[1], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[1], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[5], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[5], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v5(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[5] = atan2(v5(1), v5(0));

    T65 = tMatrix(th6[2], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[2], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[6], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[6], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T10 = tMatrix(th1[1], this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v6(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[6] = atan2(v6(1), v6(0));

    T65 = tMatrix(th6[3], this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
    T54 = tMatrix(th5[3], this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
    T32 = tMatrix(th3[7], this->coef.alpha[2], this->coef.d[2], this->coef.cn[2]);
    T21 = tMatrix(th2[7], this->coef.alpha[1], this->coef.d[1], this->coef.cn[1]);
    T43 = ur5::Pose(T32.matrix.inverse() * T21.matrix.inverse() * T10.matrix.inverse() * T60 * T65.matrix.inverse() * T54.matrix.inverse());
    Eigen::Vector3d v7(T43.matrix(0,0), T43.matrix(1,0), T43.matrix(2,0));
    th4[7] = atan2(v7(1), v7(0));

    // jointStateMsg.position = {-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};

    ur5::JointAngles allJoints[8];
    allJoints[0] << th1[0], th2[0], th3[0], th4[0], th5[0], th6[0];
    allJoints[1] << th1[0], th2[1], th3[1], th4[1], th5[1], th6[1];
    allJoints[2] << th1[1], th2[2], th3[2], th4[2], th5[2], th6[2];
    allJoints[3] << th1[1], th2[3], th3[3], th4[3], th5[3], th6[3];
    allJoints[4] << th1[0], th2[4], th3[4], th4[4], th5[0], th6[0];
    allJoints[5] << th1[0], th2[5], th3[5], th4[5], th5[1], th6[1];
    allJoints[6] << th1[1], th2[6], th3[6], th4[6], th5[2], th6[2];
    allJoints[7] << th1[1], th2[7], th3[7], th4[7], th5[3], th6[3];

    for(int i=0; i<8; i++) {
        ur5::Pose solution = this->computeDirect(allJoints[i]);
        Eigen::Matrix4d difference = solution.matrix - T60;
        std::cout << "[" << i << "] " << difference << "\n\n";
    }

    ja << allJoints[0];
    this->joints_current = ja;

    return ja;
}

// ur5::Position ur5::Robot::getPosition() {
//     return this->position;
// }

// ur5::Orientation ur5::Robot::getOrientation() {
//     return this->orientation;
// }

// ur5::Pose ur5::Robot::getPose() {
//     return this->pose;
// }

// bool ur5::Robot::setJointAngles(ur5::JointAngles joints) {
//     this->jointAngles = joints;

//     return true;
// }

// ur5::JointAngles ur5::Robot::getJointAngles() {
//     return this->jointAngles;
// }

// ur5::Pose ur5::Robot::computeDirect(const ur5::JointAngles &ja) {
//     ur5::Pose TM, worldAdjust;
//     ur5::Pose T[ur5::noJoints];

//     float flipAngle = M_PI;
//     worldAdjust << 1, 0, 0, 0,
//                    0, -1, 0, 0,
//                    0, 0, -1, 0,
//                    0, 0, 0, 1;

//     for(int i=0; i<ur5::noJoints; i++) {
//         T[i] = tMatrix(ja(i), alpha[i], d[i], cn[i]);
//     }

//     TM = worldAdjust * T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

//     this->orientation = TM.block<3,3>(0,0);
//     this->position = TM.block<3,1>(0,3);
//     this->pose = TM;

//     return TM;
// }

// ur5::Pose ur5::Robot::computeDirect() {
//     return this->computeDirect(this->jointAngles);
// }

// ur5::JointAngles ur5::Robot::computeInverse(const Pose &pose) {
//     ur5::JointAngles ja;
//     ja << 0, 0, 0, 0, 0, 0;

//     //𝜃1
//     ur5::Vector4d z60(0, 0, -this->coef.d6, 1);
//     ur5::Pose p50 = pose * z60;

//     double psi = atan2(p50(1), p50(0));
//     double phi = acos(this->coef.d4 / hypot(p50(0), p50(1)));
//     double theta1 = psi + phi + M_PI_2;

//     //𝜃5
//     double p60x = pose(0,3);
//     double p60y = pose(1,3);
//     double theta5 = acos( (p60x * sin(theta1) - p60y * cos(theta1) - this->coef.d4) / this->coef.d6 );

//     //𝜃6
//     double xhat_y = pose(0,1);
//     double yhat_y;
//     double theta6 = atan2(0,0);

//     //𝜃2

//     //𝜃3

//     //𝜃4

//     // jointStateMsg.position = {-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};

//     return ja;
// }

// std_msgs::Float32MultiArray ur5::Robot::motionPlanningMessage(const Position &initial, const Position &final, const int &noSteps) {
//     std_msgs::Float32MultiArray motionMsg;
//     motionMsg.data.resize(7);

//     motionMsg.data.at(0) = initial(0);
//     motionMsg.data.at(1) = initial(1);
//     motionMsg.data.at(2) = initial(2);
//     motionMsg.data.at(3) = noSteps;
//     motionMsg.data.at(4) = final(0);
//     motionMsg.data.at(5) = final(1);
//     motionMsg.data.at(6) = final(2);

//     return motionMsg;
// }

// ur5::Orientation ur5::Robot::computeOrientation(const ur5::EulerAngles &p) {
//     ur5::Orientation orientation;

//     const double roll = p(0);
//     const double pitch = p(1);
//     const double yaw = p(2);

//     orientation << cos(yaw) * cos(pitch),
//                    cos(yaw) * sin(roll) * sin(pitch) - cos(roll) * sin(yaw),
//                    cos(yaw) * cos(roll) * sin(pitch) + sin(yaw) * sin(roll),

//                    sin(yaw) * cos(pitch),
//                    sin(yaw) * sin(roll) * sin(pitch) + cos(roll) * cos(yaw),
//                    sin(yaw) * cos(roll) * sin(pitch) - sin(roll) * cos(yaw),

//                    -sin(pitch),
//                    cos(pitch) * sin(roll),
//                    cos(pitch) * cos(roll);

//     return orientation;

// }

// ur5::Vector4d P(double th1, double th5, double th6, const Eigen::MatrixXd& T60){
//     Eigen::Matrix4d T65, T54, T10;
//     Eigen::MatrixXd T61, T41;
//     Eigen::Vector4d tmp, P31;

//     T65 = tMatrix(th6, this->coef.alpha[5], this->coef.d[5], this->coef.cn[5]);
//     T54 = tMatrix(th5, this->coef.alpha[4], this->coef.d[4], this->coef.cn[4]);
//     T10 = tMatrix(th1, this->coef.alpha[0], this->coef.d[0], this->coef.cn[0]);
    
//     T61 = T10.matrix.inverse() * T60;
//     T41 = T61 * T54.matrix.inverse() * T65.matrix.inverse();
//     tmp << 0, -this->coef.d[3], 0, 1;
//     P31 = T41 * tmp;

//     return P31;   
// }