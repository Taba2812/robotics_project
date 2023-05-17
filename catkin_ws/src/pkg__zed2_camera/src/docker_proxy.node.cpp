#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

int main (int argc, char **argv) {
    ros::init(argc, argv, "Docker_Proxy");

    ros::NodeHandle handle;

    //Setup Params
    std::string in_pcl, out_pcl, in_img, out_img;
    int queue;
    handle.getParam("Docker_Data", in_pcl);
    handle.getParam("Docker_Image", in_img);
    handle.getParam("Zed2Det_Data", out_pcl);
    handle.getParam("Zed2Det_Img", out_img);
    handle.getParam("Q_Size", queue);

    ros::Publisher pcl_pub = handle.advertise<sensor_msgs::PointCloud2>(out_pcl, queue);
    ros::Publisher img_pub = handle.advertise<sensor_msgs::Image>(out_img, queue);

    auto pointcloud_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        pcl_pub.publish(point_cloud);
    };

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &image) {
        img_pub.publish(image);
    };

    ros::Subscriber pcl_sub = handle.subscribe<sensor_msgs::PointCloud2>(in_pcl, queue, pointcloud_callback);
    ros::Subscriber img_sub = handle.subscribe<sensor_msgs::Image>(in_img, queue, image_callback);

    ros::spin();

    return EXIT_SUCCESS;
}