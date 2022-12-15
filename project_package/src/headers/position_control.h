#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include <iostream>
#include "ros/ros.h"
#include "headers/direct_kinematics.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define JOINTS 6
#define QUEUE_SIZE 1
#define LOOP_RATE 100

//'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'

std::string joint_names[JOINTS] = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
Eigen::VectorXd q;

void get_position(const sensor_msgs::JointState::ConstPtr& js);
void new_pose(double x, double y, double z, double roll, double pitch, double yaw);

#endif
