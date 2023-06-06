#ifndef __UR5_H__
#define __UR5_H__

#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

//geometry constants
#define D1 0.089159
#define A2 -0.425
#define A3 -0.39225
#define D4 0.10915
#define D5 0.09465
#define D6 0.0823

//joint limits for the UR5 arm
static constexpr double joint1_min = -2.9671;
static constexpr double joint1_max = 2.9671;
static constexpr double joint2_min = -1.8326;
static constexpr double joint2_max = -0.1;
static constexpr double joint3_min = -2.617;
static constexpr double joint3_max = -0.1;
static constexpr double joint4_min = -3.1416;
static constexpr double joint4_max = 0.1;
static constexpr double joint5_min = -2.094;
static constexpr double joint5_max = 2.094;
static constexpr double joint6_min = -6.2832;
static constexpr double joint6_max = 6.2832;

#define JOINTS 6
#define LOOP_RATE 100
#define MATRIX_DIM 4
#define WAITING_TIME 500000

#define WAITING  0
#define JS       1
#define VISION   2
#define POSITION 3
#define MOTION   4
#define TO_BLOCK 5
#define TO_FINAL 6
#define HOMING   7

#define _USE_MATH_DEFINES

typedef Eigen::Vector3d BlockPosition;
typedef Eigen::Vector3d FinalDestination;
typedef Eigen::Matrix<double, JOINTS, 1> JointConfiguration;
typedef Eigen::Matrix<double, MATRIX_DIM, MATRIX_DIM> Matrix4d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

const double d[JOINTS] = {D1, 0, 0, D4, D5, D6};                  //distance between axes
const double cn[JOINTS] = {0, -A2, -A3, 0, 0, 0};                 //common normal
const double alpha[JOINTS] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};  //angles
static bool processStatus;

const ros::V_string jointNames = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
static JointConfiguration q;

#endif
