#include <iostream>
#include <fstream>
#include <sstream>
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
#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080
#define ERROR_RANGE 0.1f
#define IMG_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_img.png"
#define RAW_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_raw.txt"
#define PCL_PATH "/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/SimulatedZed2_pcl.png"

typedef pcl::PointCloud<pcl::PointXYZ> PTL_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PTL_PointCloudPtr;

void save_raw_matrix_to_txt (cv::Mat mat) {
    std::cout << "Saving Matrix..." << std::endl;

    std::ofstream raw_file;
    raw_file.open(RAW_PATH);

    for (int h = 0; h < mat.rows; h++) {
        for (int w = 0; w < mat.cols; w++) {
            for (int c = 0; c < mat.channels(); c++) {
                float val = mat.at<cv::Vec3f>(h,w)[c];
                raw_file << val << " ";
            }
            raw_file << std::endl;
        }
    }

    raw_file.close();
}

cv::Mat load_raw_matrix_from_txt (std::string path) {
    std::cout << "Loading Matrix..." << std::endl;
    
    cv::Mat tmp(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));
    std::ifstream raw_file(RAW_PATH);

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

        if (w == IMAGE_WIDTH) {w = 0;h++;} else {w++;}

        //std::cout << "h: " << h << " w: " << w << std::endl;
        tmp.at<cv::Vec3f>(h,w) = cv::Vec3f(x,y,z);
    }

    raw_file.close();

    return tmp;

}

cv::Mat adjust_raw_matrix (cv::Mat mat) {
    cv::Mat tmp(mat.rows, mat.cols, CV_32FC3, cv::Scalar(0));
    
    for (int h = 0; h < mat.rows; h++) {
        for (int w = 0; w < mat.cols; w++) {
            for (int c = 0; c < mat.channels(); c++) {
                float val = mat.at<cv::Vec3f>(h,w)[c];
                if (std::isnan(val)){
                    tmp.at<cv::Vec3f>(h,w)[c] = 0.0f;
                } else {
                    tmp.at<cv::Vec3f>(h,w)[c] = val;
                }
            }
        }
    }

    return tmp;
}

bool compare_matrices (cv::Mat mat1, cv::Mat mat2) {
    if (mat1.rows != mat2.rows) {
        std::cout << "Diverso numero di righe" << std::endl;
        return false;
    }
    if (mat1.cols != mat2.cols) {
        std::cout << "Diverso numero di colonne" << std::endl;
        return false;
    }
    if (mat1.channels() != mat2.channels()) {
        std::cout << "Diverso numero di canali" << std::endl;
        return false;
    }

    for (int h = 0; h < mat1.rows; h++) {
        for (int w = 0; w < mat1.cols; w++) {
            for (int c = 0; c < mat1.channels(); c++) {
                float val1 = mat1.at<cv::Vec3f>(h,w)[c];
                float val2 = mat2.at<cv::Vec3f>(h,w)[c];
                if (val2 > val1 + ERROR_RANGE) {
                    std::cout << "Diverso valore al punto h:" << h << " w:" << w << " c:" << c << std::endl;
                    std::cout << "Val1:" << val1 << " Val2:" << val2 << std::endl;
                    return false;
                }
                //std::cout << "Val1:" << mat1.at<cv::Vec3f>(h,w)[c] << " Val2:" << mat2.at<cv::Vec3f>(h,w)[c] << std::endl;
            }
        }
    }

    return true;
}

uint16_t custom_tonemapper(double value) {
    double new_value = 0.5f;

    if (value > 0.0f) {
        new_value = -exp(-value);
    } else if (value < 0.0f) {
        new_value = exp(value);
    }

    return (new_value/2)*255;
}

cv::Mat tonemap_matrix(cv::Mat original) {
    cv::Mat tonemapped(original.rows, original.cols, CV_32SC3, cv::Scalar(0));

    for (int h = 0; h < tonemapped.rows; h++) {
        for (int w = 0; w < tonemapped.cols; w++) {
            cv::Vec3f point = original.at<cv::Vec3f>(h,w);
            //std::cout << point[0] << " " << point[1] << " " << point[2] << std::endl;
            tonemapped.at<cv::Vec3i>(h,w) = cv::Vec3i(custom_tonemapper(point[0]),custom_tonemapper(point[1]),custom_tonemapper(point[2]));
            //tonemapped.at<cv::Vec3i>(h,w) = cv::Vec3i(0,0,0);
        }
    }

    return tonemapped;
}

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

    cv::Mat matrice(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));

    int i_pcl = 0;
    int i_img = 0;
    auto pointcloud_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        if (i_pcl) {return;}

        //Convert from sensor_msgs::PointCloud2 to cv::Mat
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*point_cloud,pcl_pc2);
        PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

        if (0 < (0 + 0.1f)) {
            std::cout << "Funziona" << std::endl;
        }

        matrice = pcl_to_Mat(temp_cloud);
        matrice = adjust_raw_matrix(matrice);

        save_raw_matrix_to_txt(matrice);
        //cv::imwrite(RAW_PATH, matrice);
        //cv::Mat tonemapped = tonemap_matrix(matrice);
        //cv::imwrite(PCL_PATH, tonemapped);
        
        cv::Mat comparison = load_raw_matrix_from_txt(RAW_PATH);

        /*
        if (compare_matrices(matrice, comparison)) {
            std::cout << "Le due matrici CORRISPONDONO" << std::endl;
        } else {
            std::cout << "Le due matrici NON CORRISPONDONO" << std::endl;
        }*/

        
        while (true) {
            cv::imshow("Original", matrice);
            cv::imshow("From file", comparison);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                i_pcl++;
                break;
            }
        }

        std::cout << "Over" << std::endl;
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

    return EXIT_SUCCESS;
}