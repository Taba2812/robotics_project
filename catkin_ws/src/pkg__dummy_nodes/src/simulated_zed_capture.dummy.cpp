#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

void print_pointcloud(PTL_PointCloud cloud) {
    for (int h = 0; h < cloud.height; h++) {
        for (int w = 0; w < cloud.width; w++) {
            pcl::PointXYZ point = cloud.at(w,h);
        }
    }
}

PTL_PointCloud load_raw_matrix_from_txt (std::string path) {
    
    PTL_PointCloud tmp(IMAGE_WIDTH, IMAGE_HEIGHT, pcl::PointXYZ());
    std::ifstream raw_file(path);

    if (!raw_file.is_open()) {
        return tmp;
    }

    std::string line; int h = 0, w = 0;
    float x,y,z;
    while (std::getline(raw_file, line)) {
        raw_file >> x >> y >> z;

        tmp.at(w,h)= pcl::PointXYZ(x,y,z);

        if (w == IMAGE_WIDTH - 1) {w = 0;h++;} else {w++;}
        
    }

    raw_file.close();
    return tmp;

}


int main (int argc, char **argv) {
    ros::init(argc, argv, "SimulatedCapture_Dummy");

    ros::NodeHandle handle;

    //setup params
    std::string pub_pcl, pub_img, sub_req;
    std::string IMG_PATH, RAW_PATH;
    int queue;
    handle.getParam("Zed2Det_Data", pub_pcl);
    handle.getParam("Zed2Det_Img", pub_img);
    handle.getParam("Det2Zed_Req", sub_req);
    handle.getParam("IMG_PATH", IMG_PATH);
    handle.getParam("RAW_PATH", RAW_PATH);
    handle.getParam("Q_Size", queue);

    ros::Publisher pcl_pub = handle.advertise<sensor_msgs::PointCloud2>(pub_pcl, queue);
    ros::Publisher img_pub = handle.advertise<sensor_msgs::Image>(pub_img, queue);

    auto detection_callback = [&] (const std_msgs::BoolConstPtr &result) {
        //if (!(result->data)) {return;}
        std::cout << "[Zed2][Dummy] Recieved Request for Data" << std::endl; 

        //Prendere Pointcloud
        PTL_PointCloud pcl = load_raw_matrix_from_txt(RAW_PATH);
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(pcl, pcl_msg);
        
        //Prendere immagine
        cv::Mat img;
        img = cv::imread(IMG_PATH);
        
        if (img.empty()) {
            std::cout << "[Zed2][Dummy] Failed to open image from file" << std::endl;
        }
  
        std_msgs::Header header;
        header.frame_id = 1;
        header.seq = 1;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, img);

        std::cout << "[Zed2][Dummy] Sending new Data to Detection" << std::endl; 
        img_pub.publish(img_bridge.toImageMsg());
        pcl_pub.publish(pcl_msg);
    };

    ros::Subscriber detection_sub = handle.subscribe<std_msgs::Bool>(sub_req, queue, detection_callback);

    ros::spin();

    return EXIT_SUCCESS;
}