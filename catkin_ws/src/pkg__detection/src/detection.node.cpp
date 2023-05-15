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

#define Q_SIZE 1000

#define CAMERA_CH_SEND     "Camera_Request"
#define POINTCLOUD_CH_RCVE "Camera_Data"
#define IMAGE_CH_RCVE      "Camera_Image"
#define MAIN_CH_SEND       "Detection_Result"
#define MAIN_CH_RCVE       "Detection_Request"

int main (int argc, char **argv) {

    ros::init(argc, argv, "Detection_Node");

    bool pcl_available = false;
    cv::Mat pcl_mat (IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));

    bool img_available = false;
    cv::Mat img_mat (IMAGE_HEIGHT, IMAGE_WIDTH,  CV_8UC3, cv::Scalar(0)); 

    ros::NodeHandle nh;

    ros::Publisher camera_pub = nh.advertise<std_msgs::Bool>(CAMERA_CH_SEND, Q_SIZE);
    ros::Publisher main_pub = nh.advertise<std_msgs::Float32MultiArray>(MAIN_CH_SEND, Q_SIZE);

    auto detection = [&] () {
        std::cout << "[Detection] Running Detection..." << std::endl;
        std::cout << "[Detection] " << std::getenv("test_variable") << std::endl;
        //return Detection::Detect(pcl_mat, img_mat);
        return cv::Vec3f(7,8,9);
    };
    
    auto pcl_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        std::cout << "Recieved Point Cloud..." << std::endl;
        
        pcl_mat = DataTypeHandler::PointCloud2Mat(point_cloud);
        pcl_available = true;
        
        std::cout << "PCL Detection Condition: " << (pcl_available && img_available) << std::endl;

        if (pcl_available && img_available) {
            cv::Vec3f result = detection();
            
            std_msgs::Float32MultiArray payload;
            payload.data = {result[0], result[1], result[2]};
        
            main_pub.publish(payload);
        }
    };
    
    auto img_callback = [&] (const sensor_msgs::ImageConstPtr &image) {
        std::cout << "Recieved Image..." << std::endl;

        img_mat = DataTypeHandler::Image2Mat(image);
        img_available = true;

        std::cout << "IMG Detection Condition: " << (pcl_available && img_available) << std::endl;

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
        pcl_mat = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));
        img_mat = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH,  CV_8UC3, cv::Scalar(0));

        //Send request for new data
        std::cout << "[Detection] Requesting new Camera Data" << std::endl; 
        std_msgs::Bool reply;
        reply.data = true;
        camera_pub.publish(reply);
    };

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>(POINTCLOUD_CH_RCVE, Q_SIZE, pcl_callback);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>(IMAGE_CH_RCVE, Q_SIZE, img_callback);
    ros::Subscriber rec_sub = nh.subscribe<std_msgs::Bool>(MAIN_CH_RCVE, Q_SIZE, request_callback);

    ros::spin();
    
    return EXIT_SUCCESS;

}