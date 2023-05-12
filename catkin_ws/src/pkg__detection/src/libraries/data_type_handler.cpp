#include "data_type_handler.h"

cv::Mat DataTypeHandler::PointCloud2Mat(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    //Convert from sensor_msgs::PointCloud2 to cv::Mat
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*point_cloud,pcl_pc2);
    PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //matrice = pcl_to_Mat(temp_cloud);
    cv::Mat pcm(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));
    int i = 0; int valid_counter = 0;
    for (int h = 0; h < IMAGE_HEIGHT; h++) {
        for (int w = 0; w < IMAGE_WIDTH; w++) {
            pcl::PointXYZ pcl_point = temp_cloud->points[i];
            cv::Vec3f point(pcl_point.x, pcl_point.y, pcl_point.z);
            
            for (int c = 0; c < 3; c++) {
                if (std::isnan(point[c])){
                    point[c] = 0.0f;
                }
            }
            
            pcm.at<cv::Vec3f>(h,w) = point;
            i++;
        }
    }
}

sensor_msgs::PointCloud2 DataTypeHandler::File2PointCloud(std::string path) {
    std::cout << "Loading Matrix..." << std::endl;
    
    PTL_PointCloud tmp(IMAGE_WIDTH, IMAGE_HEIGHT, pcl::PointXYZ());
    std::ifstream raw_file(path);

    if (!raw_file.is_open()) {
        std::cout << "Error in opening file" << std::endl;
        return;
    }

    std::string line; int h = 0, w = 0;
    float x,y,z;
    std::cout << "Starting Parsing..." << std::endl;
    while (std::getline(raw_file, line)) {
        raw_file >> x >> y >> z;

        tmp.at(w,h)= pcl::PointXYZ(x,y,z);

        if (w == IMAGE_WIDTH - 1) {w = 0;h++;} else {w++;}     
    }

    raw_file.close();
    std::cout << "Closing File" << std::endl;
    
    std::cout << "Point cloud size: " << tmp.size() << std::endl;
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(tmp, pcl_msg);

    return pcl_msg;
}

cv::Mat DataTypeHandler::Image2Mat(const sensor_msgs::ImageConstPtr &image) {
    //Convert from sensor_msg::Image to cv::Mat
    cv_bridge::CvImagePtr ImgPtr;
    try {
        ImgPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    return ImgPtr->image;
}

sensor_msgs::ImagePtr DataTypeHandler::Mat2Image(cv::Mat cvMat) {
    
    if (cvMat.empty()) {
        std::cout << "Failed to open image from file" << std::endl;
    }

    std_msgs::Header header;
    header.frame_id = 1;
    header.seq = 1;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, cvMat);

    return img_bridge.toImageMsg();
}