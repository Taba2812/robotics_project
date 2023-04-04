#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define RATIO 1000
#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720
#define IMG_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_img.png"
#define PCL_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_pcl.png"

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

cv::Mat pcl_to_Mat(const PTL_PointCloudPtr point_cloud) {
    //cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC4, cv::Scalar(0));
    cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));

    /*
    for (int h = 0; h < IMAGE_HEIGHT; h++) {
        for (int w = 0; w < IMAGE_WIDTH; w++) {
            pcl::PointXYZ pcl_point = point_cloud->points[h + w];
            float distance = sqrt(pow(pcl_point.x,2) + pow(pcl_point.y,2) + pow(pcl_point.z,2));
            cv::Vec4f point(pcl_point.x, pcl_point.y, pcl_point.z, distance);
            //std::cout << "h: " << h << " w: " << 2 << " Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
            pcm.at<cv::Vec4f>(h,w) = point;
        }
    }
    */
    int i = 0;
    int valid_counter = 0;
    for (int h = 0; h < IMAGE_HEIGHT; h++) {
        for (int w = 0; w < IMAGE_WIDTH; w++) {
            pcl::PointXYZ pcl_point = point_cloud->points[h+w];
            //float distance = sqrt(pow(pcl_point.x,2) + pow(pcl_point.y,2) + pow(pcl_point.z,2));
            cv::Vec3f point(pcl_point.x, pcl_point.y, pcl_point.z);
            //std::cout << "h: " << h << " w: " << 2 << " Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
            /*
            if (!std::isnan(point[0])) {
                std::cout << "i: " << valid_counter << " h: " << h << " w: " << w << " x: " << point[0] << std::endl;
                valid_counter++;
            }*/
            
            pcm.at<cv::Vec3f>(h,w) = point;
            i++;
        }
    }

    return pcm;
}

void print_matrix (cv::Mat matrice) {
    for (int h = 0; h < 1; h++) {
        for (int w = 0; w < 512; w++) {
            cv::Vec4f point = matrice.at<cv::Vec4f>(h,w);
            std::cout << "h: " << h << " w: " << 2 << " Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
        }
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "SimulatedZed_Dummy");

    ros::NodeHandle handle;

    int i_pcl = 0;
    int i_img = 0;
    auto pointcloud_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        if (i_pcl) {return;}

        //Convert from sensor_msgs::PointCloud2 to cv::Mat
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*point_cloud,pcl_pc2);
        PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


        cv::Mat matrice = pcl_to_Mat(temp_cloud);
        cv::imwrite(PCL_PATH, matrice);
        cv::imshow("Window", matrice);

        int c = cv::waitKey(10);
        if (c == 'k') {
            return;
        }

        i_pcl++;
    };

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &image) {
        if (i_img) {return;}

        //Convert from sensor_msg::Image to cv::Mat
        cv_bridge::CvImagePtr ImgPtr;
        try {
            ImgPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::imwrite(IMG_PATH, ImgPtr->image);

        i_img++;
    };

    ros::Subscriber pcl_sub = handle.subscribe<sensor_msgs::PointCloud2>("/ur5/zed_node/point_cloud/cloud_registered", RATIO, pointcloud_callback);
    ros::Subscriber img_sub = handle.subscribe<sensor_msgs::Image>("/ur5/zed_node/left_raw/image_raw_color", RATIO, image_callback);

    ros::spin();

    return 1;
}