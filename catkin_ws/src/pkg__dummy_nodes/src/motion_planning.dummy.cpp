#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

#define RATIO 1000

int main (int argc, char **argv) {
    ros::init(argc, argv, "MotionPlanning_Dummy");

    ros::NodeHandle handle;

    ros::Publisher data_publisher = handle.advertise<std_msgs::Float32MultiArray>("Motion_Planning_Data", RATIO);
    ros::Publisher request_publisher = handle.advertise<std_msgs::Bool>("Motion_Planning_Request", RATIO);

    std_msgs::Float32MultiArray destination;
    destination.data = {6,6,6,20};

    int i = 0;
    auto progress_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {
        i++;
        std::cout << "Step number: " << i << " point: x" << next_position->data[0] << " y: " << next_position->data[1] << " z: " << next_position->data[2] << std::endl;

        if (next_position->data[0] == destination.data[0] && next_position->data[1] == destination.data[1] && next_position->data[2] == destination.data[2]) {
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            data_publisher.publish(reset);
        } else {
            std_msgs::Bool next;
            next.data = true;
            request_publisher.publish(next);
        }
    };

    ros::Subscriber data_subscriber = handle.subscribe<std_msgs::Float32MultiArray>("Motion_Planning_Progress", RATIO, progress_callback);

    std::cout << "Press any key ";
    std::cin.get();

    data_publisher.publish(destination);

    ros::spin();

    return 0;
}