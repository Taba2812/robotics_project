#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>

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

    //Expected result calculation
    std::vector<float> block_position = {0.64, 0.6, 0.95};
    std::vector<float> camera_position = {1.0, 3.2, 2.2};
    std::vector<float> difference = {block_position[0]-camera_position[0],
                                     block_position[1]-camera_position[1],
                                     block_position[2]-camera_position[2]};
    float ex_dist = sqrt((difference[0]*difference[0])+(difference[1]*difference[1])+(difference[2]*difference[2]));

    auto detection_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &result) {
        float a = result->data[0] * result->data[0];
        float b = result->data[1] * result->data[1];
        float c = result->data[2] * result->data[2];
        float tot = a + b + c;
        float dist = sqrt(tot);
        std::cout <<  "[Core][Dummy] Block Detected at position: [" << result->data[0] << "," << result->data[1] << "," << result->data[2] << "] Distance: " << dist << std::endl;
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Float32MultiArray>(det_res, queue, detection_callback);

    std::cout << "[Core][Dummy] Press any key to send a detection request: ";
    std::cin.get();

    std::cout << std::endl << "[Core][Dummy] Sending Detection Request" << std::endl;
    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);

    ros::spin();
}