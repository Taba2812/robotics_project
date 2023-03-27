#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "../../shared_libraries/temp_file_handler.h"
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define RATIO 1000
#define FAKE_PCL_MAT_PATH "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/complete_data_examples/ExampleMatrix_SingleBlock"
#define FAKE_PNG_MAT_PATH "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/complete_data_examples/Example_Image_Color_SingleBlock.png"

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;


int main (int argc, char **argv) {
    ros::init(argc, argv, "Detection_Dummy");

    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<std_msgs::Bool>("Camera_Data", RATIO);

    auto camera_data_callback = [&] (const std_msgs::BoolConstPtr &request) {
        if (!(request->data)) {return;}

        cv::Mat pcl;
        TempFileHandler::LoadMatBinary(FAKE_PCL_MAT_PATH, pcl);
    };

    ros::Subscriber sub = handle.subscribe<std_msgs::Bool>("Camera_Request", RATIO, camera_data_callback);

    ros::spin();
}