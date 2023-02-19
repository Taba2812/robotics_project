#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "libraries/temp_file_handler.h"
#include "libraries/detection.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

#define CAMERA_CH_SEND "Camera_Bool"
#define CAMERA_CH_RCVE "Camera_Pcl"
#define MAIN_CH_SEND "Main_MultiArray"
#define MAIN_CH_RCVE "Main_Bool"

#define PNG_PATH ""

#define Q_SIZE 1000

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

enum State { Idle, Sending, Waiting };

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

void print_matrix (cv::Mat matrice) {
    for (int h = 0; h < 1; h++) {
        for (int w = 0; w < 512; w++) {
            cv::Vec4f point = matrice.at<cv::Vec4f>(h,w);
            std::cout << "Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
        }
    }
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "CPP_Listener");

    ros::NodeHandle rec_handle;

    ros::Publisher camera_pub = rec_handle.advertise<std_msgs::Bool>(CAMERA_CH_SEND, Q_SIZE);
    ros::Publisher main_pub = rec_handle.advertise<std_msgs::Float64MultiArray>(MAIN_CH_SEND, Q_SIZE);

    auto main_callback = [&](const std_msgs::BoolConstPtr &request) {
        if (!request->data) {return;}
        camera_pub.publish(true);
    };

    auto camera_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    
        std::cout << "Package Received" << std::endl;

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*point_cloud,pcl_pc2);
        PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

        std::cout << "Width: " << temp_cloud->width << " Height: " << temp_cloud->height << std::endl; 

        cv::Mat mat_PCL = pcl_to_Mat(temp_cloud);
        cv::Mat mat_PNG = cv::imread(PNG_PATH);

        //Block detection 
        cv::Vec3f result;
        result = Detection::Detect(mat_PCL, mat_PNG);
        
        //Send data to main
        //Convert Vec3f to Float_MultiArray
        std_msgs::Float64MultiArray payload;
        payload.data[0] = result[0];
        payload.data[1] = result[1];
        payload.data[2] = result[2];
        
        main_pub.publish(payload);

    };   

    ros::Subscriber camera_sub = rec_handle.subscribe<sensor_msgs::PointCloud2>(CAMERA_CH_RCVE, Q_SIZE, camera_callback);
    ros::Subscriber main_sub = rec_handle.subscribe<std_msgs::Bool>(MAIN_CH_RCVE, Q_SIZE, main_callback);

    ros::spin();
    
    return 0;

}