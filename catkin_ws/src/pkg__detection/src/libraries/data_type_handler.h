#ifndef DATA_HANDLER
#define DATA_HANDLER

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "settings.h"

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

namespace DataTypeHandler {

    cv::Mat PointCloud2Mat(const sensor_msgs::PointCloud2ConstPtr &point_cloud);
    sensor_msgs::PointCloud2 File2PointCloud(std::string path);

    cv::Mat Image2Mat(const sensor_msgs::ImageConstPtr &image);
    sensor_msgs::ImagePtr Mat2Image(cv::Mat cvMat);

    cv::Mat ExpensiveCrop(cv::Mat cvMat, cv::Rect rect);

}

#endif