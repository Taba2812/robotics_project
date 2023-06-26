#include "data_type_handler.h"

cv::Mat DataTypeHandler::PointCloud2Mat(const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
    //Convert from sensor_msgs::PointCloud2 to cv::Mat
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*point_cloud,pcl_pc2);
    PTL_PointCloudPtr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //matrice = pcl_to_Mat(temp_cloud);
    cv::Mat pcm(setting::access.IMAGE_HEIGHT, setting::access.IMAGE_WIDTH, CV_32FC3, cv::Scalar(0));

    auto lambda = [&] (f32_Pixel &pixel, const int *position) {
        int i = (position[1] * position[0]) + position[1];
        pcl::PointXYZ pcl_point = temp_cloud->points[i];
        cv::Vec3f point(pcl_point.x, pcl_point.y, pcl_point.z);

        for (int c = 0; c < 3; c++) {
            if (std::isnan(point[c])){
                point[c] = 0.0f;
            }
        }

        pixel = point;
    };

    pcm.forEach<f32_Pixel>(lambda);

    return pcm;
}

sensor_msgs::PointCloud2 DataTypeHandler::File2PointCloud(std::string path) {
    std::cout << "Loading Matrix..." << std::endl;
    
    PTL_PointCloud tmp(setting::access.IMAGE_WIDTH, setting::access.IMAGE_HEIGHT, pcl::PointXYZ());
    sensor_msgs::PointCloud2 pcl_msg;
    std::ifstream raw_file(path);

    if (!raw_file.is_open()) {
        std::cout << "Error in opening file" << std::endl;
        return pcl_msg;
    }

    std::string line; int h = 0, w = 0;
    float x,y,z;
    std::cout << "Starting Parsing..." << std::endl;
    while (std::getline(raw_file, line)) {
        raw_file >> x >> y >> z;

        tmp.at(w,h)= pcl::PointXYZ(x,y,z);

        if (w == setting::access.IMAGE_WIDTH - 1) {w = 0;h++;} else {w++;}     
    }

    raw_file.close();
    std::cout << "Closing File" << std::endl;
    
    std::cout << "Point cloud size: " << tmp.size() << std::endl;
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
        return ImgPtr->image;
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

cv::Mat DataTypeHandler::ExpensiveCrop(cv::Mat cvMat, cv::Rect rect) {
    cv::Mat pcm(rect.height, rect.width, CV_32FC3, cv::Scalar(0));

    for (int h = 0; h < pcm.rows; h++) {
        for (int w = 0; w < pcm.cols; w++) {          
            
        }
    }

    return pcm;
}