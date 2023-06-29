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
    float x,y,z, a, b, c;
    handle.getParam("Core2Det_Req", det_req);
    handle.getParam("Det2Core_Res", det_res);
    handle.getParam("Q_Size", queue);
    handle.getParam("X", x);
    handle.getParam("Y", y);
    handle.getParam("Z", z);
    handle.getParam("A", a);
    handle.getParam("B", b);
    handle.getParam("C", c);
    
    ros::Publisher pub = handle.advertise<std_msgs::Float32MultiArray>(det_res, queue);

    bool done = false;
    auto detection_callback = [&] (const std_msgs::BoolConstPtr &result) {
        if (done) {return;}
        
        std::cout <<  "[Detection][Dummy] Received Request for Detection" << std::endl;

        std_msgs::Float32MultiArray payload;
        payload.data = {x, y, z, a, b, c};
        std::cout <<  "[Detection][Dummy] Sending Detection Position" << std::endl;
        std::cout <<  "[Detection][Dummy] Sent Block Position: X:" << x << " Y:" << y << "Z:" << z << std::endl;
        std::cout <<  "[Detection][Dummy] Sent Dest  Position: X:" << a << " Y:" << b << "Z:" << c << std::endl;
        done = true;
        pub.publish(payload);
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Bool>(det_req, queue, detection_callback);

    ros::spin();
}