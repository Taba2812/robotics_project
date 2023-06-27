#include "libraries/gazebo_interpreter.h"
#include "libraries/robot.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "Gazebo_Interpreter_Node");
    ros::NodeHandle nh;

    Gazebo::Interpreter gi;

    int queue, frequency;
    float delta;
    std::string to_gazebo, from_gazebo, to_ur5, from_ur5;

    nh.getParam("Docker_Joint_Out", to_gazebo);
    nh.getParam("Docker_Joint_In", from_gazebo);
    nh.getParam("Gazebo2UR5", to_ur5);
    nh.getParam("UR52Gazebo", from_ur5);
    nh.getParam("Q_Size", queue);
    nh.getParam("FREQUENCY", frequency);
    nh.getParam("DELTA", delta);

    ros::Rate rate(frequency);
    std::cout << "Delta: " << delta << std::endl;

    ros::Publisher pub_gazebo = nh.advertise<std_msgs::Float64MultiArray>(to_gazebo, queue);
    ros::Publisher pub_ur5 = nh.advertise<sensor_msgs::JointState>(to_ur5, queue);

    bool setJoint = false;
    int message_counter = 0;
    //Stuff
    auto l_gazebo = [&] (const sensor_msgs::JointStateConstPtr &joint_states) {
        ur5::JointAngles joints = gi.parseJointState(joint_states);
        if (gi.moving) {
            if (gi.hasReachedDestination(joints, delta)) {
                gi.moving = false;
            } else {
                std::cout << "Sent Message# " << message_counter << " --------------------------" << std::endl;
                pub_gazebo.publish(gi.publishDestination());
                rate.sleep();
                message_counter++;
            }

        } else {
            //Calculate destination;
            if (!setJoint) {
                joints[0] = M_PI_4;
                joints[1] = 0;
                joints[2] = 0;
                joints[3] = 0;
                joints[4] = 0;
                joints[5] = 0;
                setJoint = true;
            }
            gi.setDestination(joints);
            gi.moving = true;
        }
    };

    auto l_ur5 = [&] (const std_msgs::Float64MultiArrayConstPtr &joint_states) {
        
    };

    ros::Subscriber sub_gazebo = nh.subscribe<sensor_msgs::JointState>(from_gazebo, queue, l_gazebo);
    ros::Subscriber sub_ur5 = nh.subscribe<std_msgs::Float64MultiArray>(from_ur5, queue, l_ur5);

    ros::spin();

    return EXIT_SUCCESS;
}