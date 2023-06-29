#include "libraries/gazebo_interpreter.h"
#include "libraries/robot.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "Gazebo_Interpreter_Node");
    ros::NodeHandle nh;

    Gazebo::Interpreter gi;
    ur5::JointAngles current_position;
    ur5::JointAngles home;
        home[0] = 0;
        home[1] = -0.1;
        home[2] = -2.617;
        home[3] = -M_PI_2 - (M_PI / 8);
        home[4] = -M_PI_2;
        home[5] = 0;
    gi.setDestination(home);
    bool atDestination = false;
    bool waiting_ur5 = false;

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

    ros::Publisher pub_gazebo = nh.advertise<std_msgs::Float64MultiArray>(to_gazebo, queue);
    ros::Publisher pub_ur5 = nh.advertise<std_msgs::Float64MultiArray>(to_ur5, queue);

    //Stuff
    auto l_gazebo = [&] (const sensor_msgs::JointStateConstPtr &joint_states) {
        ur5::JointAngles joints = gi.parseJointState(joint_states);
        
        current_position = joints;
        if (atDestination && !waiting_ur5) {
            pub_ur5.publish(gi.createJointMessage(joints));
            waiting_ur5 = true;
            std::cout << "[Gazebo-Interface][At Destination] Forwarded Joint Status to UR5 Controller" << std::endl;
        }
    };

    auto l_ur5 = [&] (const std_msgs::Float64MultiArrayConstPtr &joint_states) {
        ur5::JointAngles joints = gi.parseArray(joint_states);

        waiting_ur5 = false;
        std::cout << "[Gazebo-Interface] Received new destination" << std::endl;
        gi.setDestination(joints);
    };

    ros::Subscriber sub_gazebo = nh.subscribe<sensor_msgs::JointState>(from_gazebo, 1, l_gazebo);
    ros::Subscriber sub_ur5 = nh.subscribe<std_msgs::Float64MultiArray>(from_ur5, queue, l_ur5);

    sleep(2);

    std::cout << "[Gazebo-Interface] Homing" << std::endl;
    while (ros::ok()) {
        atDestination = gi.hasReachedDestination(current_position, delta);
        //std::cout << "[Gazebo-Interface][Debug] atDestination=" << atDestination << std::endl;
        ros::spinOnce();

        if (!atDestination) 
            //std::cout << "[Gazebo-Interface] NOT AT DESTINATION" << std::endl;
            rate.sleep();
            pub_gazebo.publish(gi.publishDestination());

        rate.sleep();
    }

    return EXIT_SUCCESS;
}