#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
<<<<<<< HEAD
#include "sensor_msgs/PointField.h"
#include <pcl_ros/point_cloud.h>
//#include <opencv2/core.hpp>
#include <vector>

void save_to_file(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {

=======

void ros_callback(const sensor_msgs::PointCloud2 &point_cloud) {
    std::cout << "received data " << point_cloud.data[0] << std::endl;
>>>>>>> 943d46c7e21d4a60b1650f15a94494aa8ab4078d
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

void ros_callback(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    std::cout << "received data " << std::endl;

    std::cout << "Height: " << point_cloud->height << " Width: " << point_cloud->width << std::endl;
    std::cout << "Row Step: " << point_cloud->row_step << " Point Step: " << point_cloud->point_step << std::endl;

    for (auto field : point_cloud->fields) {
        std::cout << field << " ";
    }
    std::cout << std::endl;

    pc2_to_Mat(point_cloud);

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