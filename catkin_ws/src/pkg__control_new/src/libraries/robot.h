#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <string.h>
#include <iostream>
#include <Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>

#define NO_JOINTS 6

// namespace ur5 {
//     const int noJoints = 6;
//     typedef Eigen::Matrix<double, noJoints, 1> JointAngles;
//     const ros::V_string jointNames = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

//     double joint0_min = -2.9671;
//     double joint0_max = 2.9671;
//     double joint1_min = -1.8326;
//     double joint1_max = -0.1;
//     double joint2_min = -2.617;
//     double joint2_max = -0.1;
//     double joint3_min = -3.1416;
//     double joint3_max = 0.1;
//     double joint4_min = -2.094;
//     double joint4_max = 2.094;
//     double joint5_min = -6.2832;
//     double joint5_max = 6.2832;

//     typedef Eigen::Vector3d Position;
//     typedef Eigen::Matrix3d Orientation;
//     typedef Eigen::Vector3d EulerAngles;
//     typedef Eigen::Matrix<double, 4, 4> Pose;
//     typedef Eigen::Matrix<double, 4, 1> Vector4d;

//     const float a2 = -0.4250;
//     const float a3 = -0.3922;
//     const float d1 = 0.1625;
//     const float d4 = 0.1333;
//     const float d5 = 0.0997;
//     const float d6 = 0.0996;

//     const float d[noJoints] = {d1, 0.0f, 0.0f, d4, d5, d6};
//     const float cn[noJoints] = {0.0f, 0.0f, a2, a3, 0.0f, 0.0f};
//     const float alpha[noJoints] = {0.0f, M_PI_2, 0.0f, 0.0f, M_PI_2, -M_PI_2};

//     class Robot {
//         private:
//             JointAngles jointAngles;
//             Position position;
//             Orientation orientation;
//             Pose pose;

//             Pose world_displacement;
//         public:
//             Robot();

//             Position getPosition();
//             Orientation getOrientation();
//             Pose getPose();
//             JointAngles getJointAngles();
//             bool setJointAngles(JointAngles);

//             Pose computeDirect(const JointAngles &ja);
//             Pose computeDirect();
//             JointAngles computeInverse(const Pose &pose);
//             std_msgs::Float32MultiArray motionPlanningMessage(const Position &initial, const Position &final, const int &noSteps);
//             ur5::Orientation computeOrientation(const EulerAngles &p);
//     };
// }

namespace ur5 {
    typedef Eigen::Matrix<double, NO_JOINTS, 1> JointAngles;
    typedef Eigen::Vector3d Position;
    typedef Eigen::Matrix3d Orientation;
    typedef Eigen::Vector3d EulerAngles;
    typedef Eigen::Matrix<double, 4, 1> Vector4d;

    //TODO refactor to have inside robot class
    const ros::V_string jointNames = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
    
    struct JointLimits {
        double max[NO_JOINTS];
        double min[NO_JOINTS];
    };

    struct Ur5Coefficients {
        double alpha[NO_JOINTS];
        double cn[NO_JOINTS];
        double d[NO_JOINTS];
    };

    class Pose {
        public:
            Eigen::Matrix4d matrix;
            Eigen::Vector3d getPosition() const { return this->matrix.block<3,1>(0,3); }
            // Eigen::Vector3d getOrientation() { return this->matrix.block<3,3>(0,0); }
            Eigen::Matrix3d getOrientationMatrix() const { return this->matrix.block<3,3>(0,0); }

            Pose();
            Pose(Eigen::Matrix4d m);
            Pose(Eigen::Vector3d position, Eigen::Vector3d orientation);
            Pose(Eigen::Vector3d position, Eigen::Matrix3d orientation);

            Pose operator*(Eigen::Matrix4d m) {
                return Pose(this->matrix * m);
            }

            Pose operator*(Pose p) {
                return Pose(this->matrix * p.matrix);
            }
    };

    class Robot {
        private:
            JointAngles joints_current;
            JointAngles home;
            JointLimits joints_limits;
            Ur5Coefficients coef;
            Pose world_displacement;
            Pose ee;
            //Pose eeWorld;

            JointAngles selectBestJoints(std::vector<JointAngles> joints);
            bool respectsJointsRestrictions(const JointAngles joint); 
            float calculateDistance(JointAngles neAngles, JointAngles current);

        public:

            Pose computeDirect();
            Pose computeDirect(const JointAngles ja);
            JointAngles computeInverse(const Pose pose);

            JointAngles getCurrentJoints() {return this->joints_current;}
            Eigen::Vector3d getEE_position() {return ee.getPosition();}
            Eigen::Vector3d getHomePosition() {return (this->computeDirect(this->home)).getPosition();}
            JointAngles getHomeJoints() {return this->home;}
            Eigen::Matrix3d getEE_orientation_matrix() {return ee.getOrientationMatrix();}
            Pose getEE() {return ee;}

            Pose translateEndEffector(Eigen::Vector3d);

            Robot();

    };
}

#endif
