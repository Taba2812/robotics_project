#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

#define RATIO 1000

int main (int argc, char **argv) {
    ros::init(argc, argv, "Core-Detection_Dummy");

    ros::NodeHandle handle;

    //setup parameters
    std::string det_req, det_res;
    int queue;
    handle.getParam("Core2Det_Req", det_req);
    handle.getParam("Det2Core_Res", det_res);
    handle.getParam("Q_Size", queue);
    
    ros::Publisher pub = handle.advertise<std_msgs::Bool>(det_req, queue);

    auto detection_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &result) {
        std::cout <<  "[Core][Dummy] Block Detected at position: [" << result->data[0] << "," << result->data[1] << "," << result->data[2] << "]" << std::endl;
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Float32MultiArray>(det_res, queue, detection_callback);

    std::cout << "[Core][Dummy] Press any key to send a detection request: ";
    std::cin.get();

    std::cout << "[Core][Dummy] Sending Detection Request" << std::endl;
    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);

    ros::spin();
}