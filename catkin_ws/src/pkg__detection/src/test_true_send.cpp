#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main (int argc, char **argv) {
    ros::init(argc, argv, "Fake_True_Sender");

    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<std_msgs::Bool>("Main_Bool", 1000);

    std::cout << "Press any key ";
    std::cin.get();

    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);
}