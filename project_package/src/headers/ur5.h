#ifndef __UR5_H__
#define __UR5_H__

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define A2 0.425
#define A3 0.39225
#define D1 0.089159
#define D4 0.10915
#define D5 0.09465
#define D6 0.0823
#define JOINTS 6
#define LOOP_RATE 100
#define MATRIX_DIM 4
#define QUEUE_SIZE 1
#define _USE_MATH_DEFINES

typedef Eigen::Vector3d BlockPosition;
typedef Eigen::Matrix<double, JOINTS, 1> JointConfiguration;
typedef Eigen::Matrix<double, MATRIX_DIM, MATRIX_DIM> Matrix4d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

double d[JOINTS] = {D1, 0, 0, D4, D5, D6};                  //distance between axes
double cn[JOINTS] = {0, -A2, -A3, 0, 0, 0};                 //common normal
double alpha[JOINTS] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};  //angles
JointConfiguration q;

#endif
