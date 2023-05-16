#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

#define RATIO 1000

int main (int argc, char **argv) {
    ros::init(argc, argv, "Detection_Dummy");

    ros::NodeHandle handle;

    
    ros::Publisher pub = handle.advertise<std_msgs::Bool>("Detection_Request", RATIO);

    auto detection_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &result) {
        std::cout <<  "Block Detected at position: [" << result->data[0] << "," << result->data[1] << "," << result->data[2] << "]" << std::endl;
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Float32MultiArray>("Detection_Result", RATIO, detection_callback);

    std::cout << "Press any key ";
    std::cin.get();

    std::cout << "[Core][Dummy] Sending Detection Request" << std::endl;
    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);

    ros::spin();
}