#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "JointState_Listener");
    ros::NodeHandle nh;

    //Setup Params
    std::string topic_name;
    nh.getParam("JointState_Listener/Topic", topic_name);

    auto messageCallback = [&] (const sensor_msgs::JointStateConstPtr &msg) {
        std::cout << "[Listener][JointState] Message of type JointState sent in topic: " << topic_name.c_str() << std::endl;
    };

    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(topic_name, 10, messageCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
