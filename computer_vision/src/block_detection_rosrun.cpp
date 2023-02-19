#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

cv::Mat pcl_to_Mat(const PTL_PointCloudPtr point_cloud) {
    cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC4, cv::Scalar(0));

    for (int h = 0; h < IMAGE_HEIGHT; h++) {
        for (int w = 0; w < IMAGE_WIDTH; w++) {
            pcl::PointXYZ pcl_point = point_cloud->points[h + w];
            float distance = sqrt(pow(pcl_point.x,2) + pow(pcl_point.y,2) + pow(pcl_point.z,2));
            cv::Vec4f point(pcl_point.x, pcl_point.y, pcl_point.z, distance);
            //std::cout << "Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
            pcm.at<cv::Vec4f>(h,w) = point;
        }
    }

    return pcm;
}

void ros_callback(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    
    std::cout << "Package Received" << std::endl;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*point_cloud,pcl_pc2);
    PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    std::cout << "Width: " << temp_cloud->width << " Height: " << temp_cloud->height << std::endl; 

    cv::Mat new_cv_mat = pcl_to_Mat(temp_cloud);

    //cv::Mat display_cloud;
    //cv::cvtColor(new_cv_mat, display_cloud,)

    //cv::imshow("PointCloudMap", new_cv_mat);

    if (i == 0) {
        SaveMatBinary("ExampleMatrix_MultipleBlock", new_cv_mat);
        i++;
    }

}


//ADD PCL Library
//CREATE FUNCTION TO TRANSLATE PointCloud2 TO CV::Mat

#define SUBSCRIBER 1

void print_matrix (cv::Mat matrice) {
    for (int h = 0; h < 1; h++) {
        for (int w = 0; w < 512; w++) {
            cv::Vec4f point = matrice.at<cv::Vec4f>(h,w);
            std::cout << "Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
        }
    }
}

int main (int argc, char **argv) {
    
    if (SUBSCRIBER) {
        ros::init(argc, argv, "CPP_Listener");
        ros::NodeHandle ros_handle;
        ros::Subscriber ros_subscriber = ros_handle.subscribe("chatter", 1000, ros_callback);

        ros::spin();
    } else {
        cv::Mat pcm(1, 512, CV_32FC4, cv::Scalar(0));
        LoadMatBinary("TestMatrixBinary", pcm);
        print_matrix(pcm);
    }
    
    //ADD SENDER AND RECIEVER FOR ORDER FROM MAIN NODE
    
    return 0;

}