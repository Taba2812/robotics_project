#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message: %s", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/my_topic", 10, messageCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
