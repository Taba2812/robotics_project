#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>

#define JOINTS 6
#define QUEUE_SIZE 1
#define LOOP_RATE 100

//'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'

std::string joint_names[JOINTS] = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
double q[JOINTS] = {0,0,0,0,0,0};

void get_position(const sensor_msgs::JointState::ConstPtr& js);

#endif
