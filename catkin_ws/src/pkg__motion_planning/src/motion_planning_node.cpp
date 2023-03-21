#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "libraries/bezier_curve.h"

#define RATIO 1000

int main (int argc, char **argv) {
    
    std::cout << "starting program" << std::endl;

    Bezier::Curve curve;

    //ROS Stuff
    ros::init(argc, argv, "MotionPlanning_Node");

    ros::NodeHandle handle;

    ros::Publisher data_publisher = handle.advertise<std_msgs::Float32MultiArray>("Motion_Planning_Progress", RATIO);


    auto next_callback = [&] (const std_msgs::BoolConstPtr &next) {
        if (!(next->data)) {return;}

        Bezier::Node progress = curve.getNext();

        std_msgs::Float32MultiArray reply;
        reply.data = {progress[0], progress[1], progress[2]};
        data_publisher.publish(reply);
    };

    auto motion_planning_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &destination) {
        //First call, generate the curve
        Bezier::Node dest = {(float)destination->data[0],(float)destination->data[1],(float)destination->data[2]};
        curve = Bezier::Curve(dest, destination->data[3]);

        if (destination->data[0] == 0.0f && destination->data[1] == 0.0f && destination->data[2] == 0.0f) {
            curve = Bezier::Curve();
        } else {
            Bezier::Node progress = curve.getNext();

            std_msgs::Float32MultiArray reply;
            reply.data = {progress[0], progress[1], progress[2]};
            data_publisher.publish(reply);
        }
    };

    ros::Subscriber data_subscriber = handle.subscribe<std_msgs::Float32MultiArray>("Motion_Planning_Data", RATIO, motion_planning_callback);
    ros::Subscriber progress_subscriber = handle.subscribe<std_msgs::Bool>("Motion_Planning_Request", RATIO, next_callback);

    std::cout << "Starting Spin" << std::endl;

    ros::spin();

    return 0;
}