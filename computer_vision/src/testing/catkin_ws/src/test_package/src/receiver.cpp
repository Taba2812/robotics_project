#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <opencv2/core.hpp>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;

void save_to_file(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {

}

void ros_callback(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*point_cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    std::cout << "Width: " << temp_cloud->width << " Height: " << temp_cloud->height << std::endl; 

    for (auto point : temp_cloud->points) {
        std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;
    }

}

std::vector<float> get_data(const sensor_msgs::PointCloud2ConstPtr &point_cloud, int row, int col) {
    //cv::Vec4f result;
    std::vector<float> result;

    int start = (point_cloud->row_step * row) + (point_cloud->point_step * 4 * col);

    for (int c = 0; c < 4; c++) {
        float value;
        uint8_t byte_array[4];
        for (int i = 0; i < 4; i++) {
            byte_array[i] = point_cloud->data[start + c + i];
        }
        std::copy(reinterpret_cast<const char*>(&byte_array[0]),
                  reinterpret_cast<const char*>(&byte_array[4]),
                  reinterpret_cast<char*>(&value));
        
        result[c] = value;
        std::cout << value << " | ";
    }

    std::cout << std::endl;

    return result;
}

std::vector<std::vector<std::vector<float>>> pc2_to_Mat(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    //cv::Mat pcm(point_cloud->height, point_cloud->width, CV_32FC4, cv::Scalar(1));
    std::vector<std::vector<std::vector<float>>> pcm;

    for (int h = 0; h < point_cloud->height; h++) {
        for (int w = 0; w < point_cloud->width; w++) {

            int distance;
            //pcm[h][w] = get_data(point_cloud, h, w);
        }
    }

    return pcm;
}


//ADD PCL Library
//CREATE FUNCTION TO TRANSLATE PointCloud2 TO CV::Mat

int main (int argc, char **argv) {
    
    ros::init(argc, argv, "CPP_Listener");
    ros::NodeHandle ros_handle;
    ros::Subscriber ros_subscriber = ros_handle.subscribe("chatter", 1000, ros_callback);

    ros::spin();
    
    return 0;

}