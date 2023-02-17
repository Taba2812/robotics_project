#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "direct_kinematics.h"
#include "motion_planning.h"
#include "state_machine/state.h"

ros::V_string joint_names = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
BlockPosition bp = BlockPosition::Zero();

void get_joint(const sensor_msgs::JointState::ConstPtr& js);
void get_position(const std_msgs::Float64MultiArray::ConstPtr& xyz);

#endif
