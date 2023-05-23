#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Float64MultiArray_Listener");
    ros::NodeHandle nh;

    //Setup Params
    std::string topic_name;
    nh.getParam("Topic", topic_name);

    auto messageCallback = [&] (const std_msgs::Float64MultiArrayConstPtr &msg) {
        std::cout << "[Listener][Float64MultiArray] Message of type Float64MultiArray sent in topic: " << topic_name.c_str() << std::endl;
    };

    ros::Subscriber sub = nh.subscribe<std_msgs::Float64MultiArray>(topic_name, 10, messageCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
