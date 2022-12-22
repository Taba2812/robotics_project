#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include <iostream>
#include "ros/ros.h"
#include "direct_kinematics.h"
#include "inverse_kinematics.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"

#define LINKS 7
#define QUEUE_SIZE 1
#define LOOP_RATE 100

std::string joint_names[JOINTS] = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
std::string link_names[LINKS] = {"base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"};
JointConfiguration q;
double p[LINKS][3];

void get_joint(const sensor_msgs::JointState::ConstPtr& js);
void get_link(const gazebo_msgs::LinkStates::ConstPtr& ls);
void new_pose(double x, double y, double z, double roll, double pitch, double yaw);

#endif
