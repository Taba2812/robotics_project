#ifndef __JOINT_SUBSCRIBER_H__
#define __JOINT_SUBSCRIBER_H__

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#define JOINTS 6
#define QUEUE_SIZE 1000
#define LOOP_FREQ 1000

void receive_jstate(sensor_msgs::JointState::ConstPtr& msg);

#endif
