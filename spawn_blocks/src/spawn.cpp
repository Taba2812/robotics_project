#include <iostream>
#include <random>
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"

#define LOOP_RATE 1

int main(int argc, char **argv){
    ros::init(argc, argv, "spawner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);

    //ros::Publisher block_pub = nh.subscribe()

    return 0;
}