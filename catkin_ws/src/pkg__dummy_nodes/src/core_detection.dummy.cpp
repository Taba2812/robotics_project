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
    float x,y,z;
    handle.getParam("Core2Det_Req", det_req);
    handle.getParam("Det2Core_Res", det_res);
    handle.getParam("Q_Size", queue);
    handle.getParam("X", x);
    handle.getParam("Y", y);
    handle.getParam("Z", z);
    
    ros::Publisher pub = handle.advertise<std_msgs::Float32MultiArray>(det_res, queue);

    auto detection_callback = [&] (const std_msgs::BoolConstPtr &result) {
        std::cout <<  "[Detection][Dummy] Recieved Request for Detection" << std::endl;

        std_msgs::Float32MultiArray payload;
        payload.data = {x, y, z};
        std::cout <<  "[Detection][Dummy] Sending Detection Position" << std::endl;
        pub.publish(payload);
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Bool>(det_req, queue, detection_callback);

    ros::spin();
}