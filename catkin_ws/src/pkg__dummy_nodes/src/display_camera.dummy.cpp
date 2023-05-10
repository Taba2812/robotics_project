#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

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
    std::cout << "width:" << IMAGE_WIDTH << " height: " << IMAGE_HEIGHT << std::endl;
    for (int h = 0; h < IMAGE_HEIGHT; h++) {
        for (int w = 0; w < IMAGE_WIDTH; w++) {
            pcl::PointXYZ pcl_point = point_cloud->points[i];
            //float distance = sqrt(pow(pcl_point.x,2) + pow(pcl_point.y,2) + pow(pcl_point.z,2));
            cv::Vec3f point(pcl_point.x, pcl_point.y, pcl_point.z);
            //std::cout << "h: " << h << " w: " << 2 << " Vx: " << point[0] << " Vy: " << point[1] << " Vz: " << point[2] << " Vd: " << point[3] << std::endl;
            
            /*
            if (!std::isnan(point[0])) {
                std::cout << "i: " << valid_counter << " h: " << h << " w: " << w << " x: " << point[0] << std::endl;
                valid_counter++;
            }
            */
            pcm.at<cv::Vec3f>(h,w) = point;
            i++;
        }
    }

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

        while (true) {
            cv::imshow("Recieved", matrice);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                break;
            }
        }        

        std::cout << "Over" << std::endl;
    };

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &result) {
        
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