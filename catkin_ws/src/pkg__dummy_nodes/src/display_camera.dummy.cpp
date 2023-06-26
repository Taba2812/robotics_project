#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"

#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#define RATIO 1000
#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080
//#define IMAGE_WIDTH 1080
//#define IMAGE_HEIGHT 720

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

typedef cv::Point3_<uint8_t> ui8_Pixel;
typedef cv::Point3_<float> f32_Pixel;

cv::Mat pcl_to_Mat(const PTL_PointCloudPtr point_cloud) {
    //cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC4, cv::Scalar(0));
    cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));

    std::cout << "width:" << IMAGE_WIDTH << " height: " << IMAGE_HEIGHT << std::endl;

    auto lambda = [&] (f32_Pixel &pixel, const int *position) {
        int i = (position[1] * position[0]) + position[1];
        pcl::PointXYZ pcl_point = point_cloud->points[i];
        cv::Vec3f point(pcl_point.x, pcl_point.y, pcl_point.z);

        pixel = point;
    };

    pcm.forEach<f32_Pixel>(lambda);

    return pcm;
}

void print_pointcloud(PTL_PointCloud cloud) {
    for (int h = 0; h < cloud.height; h++) {
        for (int w = 0; w < cloud.width; w++) {
            pcl::PointXYZ point = cloud.at(w,h);
            std::cout << "h: " << h << " w: " << w << "p: [" << point.x << "," << point.y << "," << point.z << "]" << std::endl;
        }
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "CameraDisplay_Dummy");

    ros::NodeHandle handle;
    ros::Publisher pub = handle.advertise<std_msgs::Bool>("Camera_Request", RATIO);

    auto camera_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        //Convert from sensor_msgs::PointCloud2 to cv::Mat
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*point_cloud,pcl_pc2);
        PTL_PointCloudPtr temp_cloud(new PTL_PointCloud);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

        //print_pointcloud(*temp_cloud);

        cv::Mat matrice = pcl_to_Mat(temp_cloud);

        /*
        while (true) {
            cv::imshow("Recieved", matrice);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                break;
            }
        }        
        */

        std::cout << "Over" << std::endl;
    };

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &result) {
        
        
        cv_bridge::CvImagePtr ImgPtr;
        
        try {
            //std::cout << img.height << "|" << img.width << "|" << img.step << "|" << img.data.size() << "|" << img.encoding << std::endl;
            //std::cout << img.header.frame_id << "|" << img.header.seq << "|" << img.header.stamp << std::endl;
            ImgPtr = cv_bridge::toCvCopy(result, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        
        while (true) {
            cv::imshow("Recieved", ImgPtr->image);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                break;
            }
        }        
        
    };

    ros::Subscriber sub_pcl = handle.subscribe<sensor_msgs::PointCloud2>("Camera_Data", RATIO, camera_callback);
    ros::Subscriber sub_img = handle.subscribe<sensor_msgs::Image>("Camera_Image", RATIO, image_callback);

    std::cout << "Press any key ";
    std::cin.get();

    std_msgs::Bool reply;
    reply.data = true;
    pub.publish(reply);

    std::cout << "Spinning ..." << std::endl;
    ros::spin();
}