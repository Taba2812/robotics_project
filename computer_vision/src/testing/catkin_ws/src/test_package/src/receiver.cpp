#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2"

void ros_callback(const sensor_msgs::PointCloud2 &point_cloud) {
    std::cout << "received data " << point_cloud->data[0] << std::endl;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "CPP_Listener");
    ros::NodeHandle ros_handle;
    ros::Subscriber ros_subscriber = ros_handle.subscribe("chatter", 1000, ros_callback);

    ros::spin();

    return 0;

}