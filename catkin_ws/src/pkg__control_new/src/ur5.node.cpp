#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "libraries/gazebo_interpreter.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "UR5_Node");
    ros::NodeHandle nh;

    int queue;
    nh.getParam("Q_Size", queue);

    std::cout << "Programma Parte" << std::endl;

    Gazebo::Interpreter gi;
    gi.print("Hello from the other side");

    ros::spin();
}