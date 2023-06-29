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

    //this->eeWorld = this->world_displacement * TM;
    this->ee = TM;

    return TM;
}

ur5::Pose ur5::Robot::computeDirect() {
    return this -> computeDirect(this->joints_current);
}


ur5::Robot::Robot() {
    JointLimits limits = {
        //         0        1        2       3        4        5
        /*MAX*/{ 2.9671, -0.1,    -0.1,      0.1,     2.094,  6.2832},
        /*MIN*/{-2.9671, -1.8326, -2.617,   -3.1416, -2.094, -6.2832}
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
    this->home << 0, -0.1, -2.617, -M_PI_2 - (M_PI / 8), -M_PI_2, 0;
    //In the simulation the robot is rotated 180° on the x axis
    this->world_displacement = Pose(Eigen::Vector3d::Zero(), Eigen::Vector3d(M_PI,0,0));
    this->ee = this->computeDirect();
}

ur5::Pose ur5::Robot::translateEndEffector(Eigen::Vector3d tr) {
    //Eigen::Vector4d translation(tr(0), tr(1), tr(2), 1);
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
    ur5::Pose translation(tr, id);

    Eigen::Vector3d robot_space_tr = (this->world_displacement * translation).getPosition();
    ur5::Pose robot_space_pose(robot_space_tr, id);

    return robot_space_pose * this->getEE();
}

bool ur5::Robot::respectsJointsRestrictions(const ur5::JointAngles joint) {
    bool result = true;
    for (int i = 0; i < NO_JOINTS; i++) {
        //std::cout << "joint#" << i << " is " << (joint[i] > this->joints_limits.max[i] || joint[i] < this->joints_limits.min[i]) << std::endl;
        if (joint[i] > this->joints_limits.max[i] || joint[i] < this->joints_limits.min[i]) {
            result = false;
            break;
        }
    }

    return result;
}

float ur5::Robot::calculateDistance(ur5::JointAngles newAngles, ur5::JointAngles current) {
    float dist = 0.0f;
    for (int i = 0; i < NO_JOINTS; i++) {
        dist += abs(newAngles[i] - current[i]);
    }

    return dist;
}

ur5::JointAngles ur5::Robot::selectBestJoints(std::vector<ur5::JointAngles> joints) {
    std::vector<ur5::JointAngles> best;
    std::vector<float> distances;
    for (int i = 0; i < joints.size(); i++) {
        //std::cout << "forJoints#"<< i << ": " << joints[i] << std::endl;
        if (this->respectsJointsRestrictions(joints[i])) {
            best.push_back(joints[i]);
            distances.push_back(this->calculateDistance(joints[i], this->getCurrentJoints()));
        }
    }

    int best_index = 0;

    for (int i = 0; i < (distances.size() - 1); i++) {
        if (distances[i+1] < distances[i])
            best_index = i+1;
    }

    if (best.empty()) {
        std::cout << "[Robot-Class] Inverse kinematic could not find a joints setup within the limits that the robot can endure, sending to home" << std::endl;
        return this->home;
    }

    return best[best_index];
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

    //std::cout << "T60" << std::endl;
    //std::cout << T60 << std::endl;

    //theta1
    tmp << 0, 0, -(this->coef.d[5]),1;
    P50 = T60 * tmp;
    phi = atan2(P50(1), P50(0));
    R = sqrt( pow(P50(0),2) + pow(P50(1),2) );
    psi = acos( this->coef.d[3] / R );
    th1[0] = phi + psi + M_PI_2;
    th1[1] = phi - psi + M_PI_2;

    //std::cout << "TH1_1: " << th1[0] << "\nTH1_2: " << th1[1] << "\n\n";

    Eigen::Vector3d position = pose.getPosition();

    //theta5
    th5[0] =  acos( ( position(0)*sin(th1[0]) - position(1)*cos(th1[0]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[1] = -acos( ( position(0)*sin(th1[0]) - position(1)*cos(th1[0]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[2] =  acos( ( position(0)*sin(th1[1]) - position(1)*cos(th1[1]) - this->coef.d[3] ) / this->coef.d[5] );
    th5[3] = -acos( ( position(0)*sin(th1[1]) - position(1)*cos(th1[1]) - this->coef.d[3] ) / this->coef.d[5] );

    //std::cout << "TH5_1: " << th5[0] << "\nTH5_2: " << th5[1] << "\nTH5_3: " << th5[2] << "\nTH5_4: " << th5[3] << "\n\n";

    //theta6
    T06.resize(4,4);
    T06 = T60.inverse();
    X06 << T06(0,0), T06(1,0), T06(2,0);
    Y06 << T06(0,1), T06(1,1), T06(2,1);
    th6[0] = atan2( (-X06(1)*sin(th1[0]) + Y06(1)*cos(th1[0])) / sin(th5[0]) , (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[0]) );
    th6[1] = atan2( (-X06(1)*sin(th1[0]) + Y06(1)*cos(th1[0])) / sin(th5[1]) , (X06(0)*sin(th1[0]) - Y06(0)*cos(th1[0])) / sin(th5[1]) );
    th6[2] = atan2( (-X06(1)*sin(th1[1]) + Y06(1)*cos(th1[1])) / sin(th5[2]) , (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[2]) );
    th6[3] = atan2( (-X06(1)*sin(th1[1]) + Y06(1)*cos(th1[1])) / sin(th5[3]) , (X06(0)*sin(th1[1]) - Y06(0)*cos(th1[1])) / sin(th5[3]) );

    //std::cout << "TH6_1: " << th6[0] << "\nTH6_2: " << th6[1] << "\nTH6_3: " << th6[2] << "\nTH6_4: " << th6[3] << "\n\n";

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

    //std::cout << "TH3_1: " << th3[0] << "\nTH3_2: " << th3[1] << "\nTH3_3: " << th3[2] << "\nTH3_4: " << th3[3];
    //std::cout << "\nTH3_5: " << th3[4] << "\nTH3_6: " << th3[5] << "\nTH3_7: " << th3[6] << "\nTH3_8: " << th3[7] << "\n\n";

    //theta2
    th2[0] = atan2(-p41_1(2), -p41_1(0)) - asin( (-(this->coef.cn[3]) * sin(th3[0])) / (p41xz_1) );
    th2[1] = atan2(-p41_2(2), -p41_2(0)) - asin( (-(this->coef.cn[3]) * sin(th3[1])) / (p41xz_2) );
    th2[2] = atan2(-p41_3(2), -p41_3(0)) - asin( (-(this->coef.cn[3]) * sin(th3[2])) / (p41xz_3) );
    th2[3] = atan2(-p41_4(2), -p41_4(0)) - asin( (-(this->coef.cn[3]) * sin(th3[3])) / (p41xz_4) );
    th2[4] = atan2(-p41_1(2), -p41_1(0)) - asin( ( (this->coef.cn[3]) * sin(th3[0])) / (p41xz_1) );
    th2[5] = atan2(-p41_2(2), -p41_2(0)) - asin( ( (this->coef.cn[3]) * sin(th3[1])) / (p41xz_2) );
    th2[6] = atan2(-p41_3(2), -p41_3(0)) - asin( ( (this->coef.cn[3]) * sin(th3[2])) / (p41xz_3) );
    th2[7] = atan2(-p41_4(2), -p41_4(0)) - asin( ( (this->coef.cn[3]) * sin(th3[3])) / (p41xz_4) );

    //std::cout << "TH2_1: " << th2[0] << "\nTH2_2: " << th2[1] << "\nTH2_3: " << th2[2] << "\nTH2_4: " << th2[3];
    //std::cout << "\nTH2_5: " << th2[4] << "\nTH2_6: " << th2[5] << "\nTH2_7: " << th2[6] << "\nTH2_8: " << th2[7] << "\n\n";

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

    //std::cout << "TH4_1: " << th4[0] << "\nTH4_2: " << th4[1] << "\nTH4_3: " << th4[2] << "\nTH4_4: " << th4[3];
    //std::cout << "\nTH4_5: " << th4[4] << "\nTH4_6: " << th4[5] << "\nTH4_7: " << th4[6] << "\nTH4_8: " << th4[7] << "\n\n";

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

    // for(int i=0; i<8; i++) {
    //     //Check if this compute direct returns the absolute one or the one adjusted for the robot rotation
    //     ur5::Pose solution = this->computeDirect(allJoints[i]);
    //     Eigen::Matrix4d difference = solution.matrix - T60;
    //     std::cout << "[" << i << "] " << difference << "\n\n";
    // }

    // std::cout << this->computeDirect(allJoints[0]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[1]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[2]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[3]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[4]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[5]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[6]).matrix << std::endl << std::endl;
    // std::cout << this->computeDirect(allJoints[7]).matrix << std::endl << std::endl;

    

    ja << this->selectBestJoints({allJoints[0],allJoints[1],allJoints[2],allJoints[3],allJoints[4],allJoints[5],allJoints[6],allJoints[7]});
    // std::cout << "Ja: " << ja << std::endl;
    // std::cout << "allJoints#0: " << allJoints[0] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[1] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[2] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[3] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[4] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[5] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[6] << std::endl;
    // std::cout << "allJoints#0: " << allJoints[7] << std::endl;

    this->joints_current = ja;

    return ja;
}




