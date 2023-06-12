#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include "libraries/data_type_handler.h"
#include "libraries/detection.h"

int main (int argc, char **argv) {

    ros::init(argc, argv, "Detection_Node");
    ros::NodeHandle nh;
    
    //Setting launch parameters
    std::string camera_ch_send, pointcloud_ch_rcve, image_ch_rcve, core_ch_send, core_ch_rcve;
    int image_height, image_width, queue;
    nh.getParam("Det2Zed_Req", camera_ch_send);
    nh.getParam("Zed2Det_Data", pointcloud_ch_rcve);
    nh.getParam("Zed2Det_Img", image_ch_rcve);
    nh.getParam("Det2Core_Res", core_ch_send);
    nh.getParam("Core2Det_Req", core_ch_rcve);
    nh.getParam("IMAGE_HEIGHT", image_height);
    nh.getParam("IMAGE_WIDTH", image_width);
    nh.getParam("Q_Size", queue);

    bool pcl_available = false;
    cv::Mat pcl_mat (image_height, image_width, CV_32FC3, cv::Scalar(0));

    bool img_available = false;
    cv::Mat img_mat (image_height, image_width,  CV_8UC3, cv::Scalar(0)); 

    ros::Publisher camera_pub = nh.advertise<std_msgs::Bool>(camera_ch_send, queue);
    ros::Publisher main_pub = nh.advertise<std_msgs::Float32MultiArray>(core_ch_send, queue);

    auto detection = [&] () {
        std::cout << "[Detection] Running Detection..." << std::endl;
        //return Detection::Detect(pcl_mat, img_mat);
        return cv::Vec3f(7,8,9);
    };
    
    auto pcl_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        std::cout << "[Detection] Recieved Point Cloud..." << std::endl;
        
        pcl_mat = DataTypeHandler::PointCloud2Mat(point_cloud);
        pcl_available = true;

        if (pcl_available && img_available) {
            cv::Vec3f result = detection();
            
            std_msgs::Float32MultiArray payload;
            payload.data = {result[0], result[1], result[2]};
        
            main_pub.publish(payload);
        }
    };
    
    auto img_callback = [&] (const sensor_msgs::ImageConstPtr &image) {
        std::cout << "[Detection] Received Image..." << std::endl;

        img_mat = DataTypeHandler::Image2Mat(image);
        img_available = true;

        if (pcl_available && img_available) {
            cv::Vec3f result = detection();
            
            std_msgs::Float32MultiArray payload;
            payload.data = {result[0], result[1], result[2]};
        
            main_pub.publish(payload);
        }
    };

    auto request_callback = [&] (const std_msgs::BoolConstPtr &request) {
        std::cout << "[Detection] Detection Request Recieved" << std::endl;
        if (!(request->data)) {return;}

        //Reset local data
        pcl_available = false;
        img_available = false;
        pcl_mat = cv::Mat(image_height, image_width, CV_32FC3, cv::Scalar(0));
        img_mat = cv::Mat(image_height, image_width,  CV_8UC3, cv::Scalar(0));

        //Send request for new data
        std::cout << "[Detection] Requesting new Camera Data" << std::endl; 
        std_msgs::Bool reply;
        reply.data = true;
        camera_pub.publish(reply);
    };

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_ch_rcve, queue, pcl_callback);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>(image_ch_rcve, queue, img_callback);
    ros::Subscriber rec_sub = nh.subscribe<std_msgs::Bool>(core_ch_rcve, queue, request_callback);

    ros::spin();
    
    return EXIT_SUCCESS;

}