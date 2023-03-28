#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <opencv2/core.hpp>

#define RATIO 1000

int main (int argc, char **argv) {
    ros::init(argc, argv, "CameraDisplay_Dummy");

    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<std_msgs::Bool>("Camera_Request", RATIO);

    auto camera_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &result) {
        
    };

    auto string_callback = [&] (const std_msgs::StringConstPtr &result) {
        
    };

    ros::Subscriber sub_pcl = handle.subscribe<std_msgs::Float32MultiArrayConstPtr>("Camera_Data", RATIO, camera_callback);
    ros::Subscriber sub_img = handle.subscribe<std_msgs::String>("Image_String", RATIO, string_callback);

    std::cout << "Press any key ";
    std::cin.get();

    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);

    ros::spin();
}