#include "libraries/gazebo_interpreter.h"
#include <ros/ros.h>

// const double x = 0.64;
// const double y = 0.6;
// const double z = -0.825;

int main (int argc, char** argv) {
    ros::init(argc, argv, "UR5_Node");
    ros::NodeHandle nh;

    int queue_size;
    std::string joint_docker_in, joint_docker_out;
    nh.getParam("Q_Size", queue_size);
    nh.getParam("Docker_Joint_In", joint_docker_in);
    nh.getParam("Docker_Joint_Out", joint_docker_out);

    Gazebo::Interpreter gi;

    ros::Publisher jointPub = nh.advertise<std_msgs::Float64MultiArray>(joint_docker_out, queue_size);

    bool processStatus = false;
    auto getJoint = [&] (const sensor_msgs::JointState::ConstPtr &js) {
        if (processStatus) { return; }

        gi.parseJointState(js);

        processStatus = true;
    };

    ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(joint_docker_in, queue_size, getJoint);

    int index;
    double angle;
    char c;

    ur5::Robot robot;

    while(ros::ok()) {
        std::cout << "Continue: ";
        std::cin >> c;

        while(!processStatus) ros::spinOnce();
        processStatus = false;

        std::cout << "Which joint do you want to move? ";
        std::cin >> index;

        std::cout << "How much? ";
        std::cin >> angle;

        ur5::JointAngles ja = gi.getJointAngles();
        // r.computeDirect(ja);
        ja(index) += M_PI / angle;

        Gazebo::JointMessage jm = gi.createJointMessage(ja);

        jointPub.publish(jm);
    }
}