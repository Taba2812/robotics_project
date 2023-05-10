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

/*
Riceve bool da detection
Manda immagine e pointcloud
*/

#define RATIO 1000
#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080
#define ERROR_RANGE 0.1f
#define IMG_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_img.png"
#define RAW_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_raw.txt"

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

void print_pointcloud(PTL_PointCloud cloud) {
    for (int h = 0; h < cloud.height; h++) {
        for (int w = 0; w < cloud.width; w++) {
            pcl::PointXYZ point = cloud.at(w,h);
            std::cout << "h: " << h << " w: " << w << "p: [" << point.x << "," << point.y << "," << point.z << "]" << std::endl;
        }
    }
}

PTL_PointCloud load_raw_matrix_from_txt (std::string path) {
    std::cout << "Loading Matrix..." << std::endl;
    
    PTL_PointCloud tmp(IMAGE_WIDTH, IMAGE_HEIGHT, pcl::PointXYZ());
    std::ifstream raw_file(path);

    if (!raw_file.is_open()) {
        std::cout << "Error in opening file" << std::endl;
        return tmp;
    }

    std::string line; int h = 0, w = 0;
    float x,y,z;
    std::cout << "Starting Parsing..." << std::endl;
    while (std::getline(raw_file, line)) {
        raw_file >> x >> y >> z;

        //std::cout << x << " " << y << " " << z << std::endl;
        tmp.at(w,h)= pcl::PointXYZ(x,y,z);

        if (w == IMAGE_WIDTH - 1) {w = 0;h++;} else {w++;}

        //std::cout << "h: " << h << " w: " << w << std::endl;
        
    }

    raw_file.close();
    std::cout << "returning pointcloud" << std::endl;
    return tmp;

}


int main (int argc, char **argv) {
    ros::init(argc, argv, "SimulatedCapture_Dummy");

    ros::NodeHandle handle;

    ros::Publisher pcl_pub = handle.advertise<sensor_msgs::PointCloud2>("Camera_Data", RATIO);
    ros::Publisher img_pub = handle.advertise<sensor_msgs::Image>("Camera_Image", RATIO);

    auto detection_callback = [&] (/*std_msgs::BoolConstPtr &result*/) {
        //if (!(result->data)) {return;}
   
        //Prendere Pointcloud
        PTL_PointCloud pcl = load_raw_matrix_from_txt(RAW_PATH);
        std::cout << "Point cloud size: " << pcl.size() << std::endl;
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(pcl, pcl_msg);
        
        //Prendere immagine
        cv::Mat img;
        img = cv::imread(IMG_PATH);
        
        if (img.empty()) {
            std::cout << "Failed to open image from file" << std::endl;
        }
  
        std_msgs::Header header;
        header.seq = 1;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::RGB16, img);

        img_pub.publish(img_bridge.toImageMsg());
        pcl_pub.publish(pcl_msg);
    };

    //ros::Subscriber detection_sub = handle.subscribe<std_msgs::Bool>("Camera_Request", RATIO, detection_callback);

    //ros::spin();

    detection_callback();

    return EXIT_SUCCESS;
}