#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string.h>
#include <iostream>
#include <Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>

namespace ur5 {
    const int noJoints = 6;
    typedef Eigen::Matrix<double, noJoints, 1> JointAngles;
    const ros::V_string jointNames = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

    double joint0_min = -2.9671;
    double joint0_max = 2.9671;
    double joint1_min = -1.8326;
    double joint1_max = -0.1;
    double joint2_min = -2.617;
    double joint2_max = -0.1;
    double joint3_min = -3.1416;
    double joint3_max = 0.1;
    double joint4_min = -2.094;
    double joint4_max = 2.094;
    double joint5_min = -6.2832;
    double joint5_max = 6.2832;

    typedef Eigen::Vector3d Position;
    typedef Eigen::Matrix3d Orientation;
    typedef Eigen::Matrix<double, 4, 4> Pose;
    typedef Eigen::Matrix<double, 4, 1> Vector4d;

    const float d1 = 0.089159;
    const float a2 = -0.425;
    const float a3 = -0.39225;
    const float d4 = 0.10915;
    const float d5 = 0.09465;
    const float d6 = 0.0823;

    const float d[noJoints] = {d1, 0.0f, 0.0f, d4, d5, d6};
    const float cn[noJoints] = {0.0f, a2, a3, 0.0f, 0.0f, 0.0f};
    const float alpha[noJoints] = {M_PI_2, 0.0f, 0.0f, M_PI_2, -M_PI_2, 0.0f};

    class Robot {
        private:
            JointAngles jointAngles;
            Position position;
            Orientation orientation;
            Pose destination;
        public:
            Robot();

            Position getPosition();
            Orientation getOrientation();
            JointAngles getJointAngles();

            Pose computeDirect(const JointAngles &ja);
            JointAngles computeInverse(const Pose &p);
    };
}

#endif
