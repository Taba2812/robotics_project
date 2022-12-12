#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "ros/ros.h"
#include <Eigen/Dense>
#include "joint_publisher.h"

#define JOINTS 6
#define QUEUE_SIZE 1000
#define LOOP_RATE 1000.

void get_position(const sensor_msgs::JointState::ConstPtr & msg);

#endif
