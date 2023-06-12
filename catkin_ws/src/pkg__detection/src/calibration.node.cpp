#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include "libraries/data_type_handler.h"
#include "libraries/detection.h"
#include "libraries/settings.h"

setting::Container setting::access;

void display_calibration(cv::Mat img, cv::Mat pcl) {
        //Color calibration...
        
        cv::Size new_size = cv::Size(img.size()/2);
        cv::namedWindow("Crop Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Crop Calibration", new_size.width, new_size.height);

        cv::namedWindow("Color Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Color Calibration", new_size.width, new_size.height);

        cv::namedWindow("Mask Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Mask Calibration", new_size.width, new_size.height);

        int x_start, y_start, x_end, y_end;
        int crop;
        cv::createTrackbar("x_start", "Crop Calibration", &x_start, img.cols);
        cv::createTrackbar("x_end", "Crop Calibration", &x_end, img.cols);
        cv::createTrackbar("y_start", "Crop Calibration", &y_start, img.rows);
        cv::createTrackbar("y_end", "Crop Calibration", &y_end, img.rows);
        cv::createTrackbar("crop", "Crop Calibration", &crop, 2);

        int upper_h, upper_s, upper_b;
        int lower_h, lower_s, lower_b;
        cv::createTrackbar("upper_h", "Color Calibration", &upper_h, 255);
        cv::createTrackbar("lower_h", "Color Calibration", &lower_h, 255);
        cv::createTrackbar("upper_s", "Color Calibration", &upper_s, 255);
        cv::createTrackbar("lower_s", "Color Calibration", &lower_s, 255);
        cv::createTrackbar("upper_b", "Color Calibration", &upper_b, 255);
        cv::createTrackbar("lower_b", "Color Calibration", &lower_b, 255);

        int erosion, dilation;
        cv::createTrackbar("erosion", "Mask Calibration", &erosion, 10);
        cv::createTrackbar("dilation", "Mask Calibration", &dilation, 10);

        while (true) {

            cv::Mat tmp_rect = img.clone();
            cv::Rect edges = cv::Rect(x_start, y_start, x_end, y_end);  
            cv::rectangle(tmp_rect, edges, cv::Scalar( 0, 255, 255 ));
            cv::imshow("Crop Calibration", tmp_rect);

            cv::Mat tmp_color = img.clone();
            cv::cvtColor(tmp_color, tmp_color, cv::COLOR_BGR2HSV);
            cv::inRange(tmp_color, cv::Scalar(lower_h, lower_s, lower_b), cv::Scalar(upper_h, upper_s, upper_b), tmp_color);
            cv::imshow("Color Calibration", tmp_color);

            cv::Mat tmp_mask = tmp_color.clone();
            cv::Mat tmp_mask_2;
            
            cv::Mat erosion_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*erosion+1,2*erosion+1),cv::Point(erosion,erosion));
            cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*dilation+1,2*dilation+1),cv::Point(dilation,dilation));
                                                                              
            cv::dilate(tmp_mask, tmp_mask_2, dilation_kernel);
            cv::erode(tmp_mask_2, tmp_mask, erosion_kernel);
            cv::erode(tmp_mask, tmp_mask_2, erosion_kernel);
            cv::dilate(tmp_mask_2, tmp_mask, dilation_kernel);
            
            if (crop == 1) {
                cv::imshow("Mask Calibration", tmp_mask(edges));
            } else {
                cv::imshow("Mask Calibration", tmp_mask);
            }
            

            int c = cv::waitKey(10);
            if (c == 'k') {
                break;
            }
        }
        
        cv::destroyAllWindows();
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "Calibration_Node");
    ros::NodeHandle nh;

    //Setting launch parameters
    setting::access.SetParameters(nh);
    std::string camera_ch_send, pointcloud_ch_rcve, image_ch_rcve, core_ch_send, core_ch_rcve;
    int image_height, image_width, queue;
    nh.getParam("Det2Zed_Req", camera_ch_send);
    nh.getParam("Zed2Det_Data", pointcloud_ch_rcve);
    nh.getParam("Zed2Det_Img", image_ch_rcve);
    nh.getParam("Det2Core_Res", core_ch_send);
    nh.getParam("Core2Det_Req", core_ch_rcve);
    nh.getParam("IMAGE_HEIGHT", image_height);
    nh.getParam("IMAGE_WIDTH", image_width);
    nh.getParam("Q_Size", queue);


    bool pcl_available = false;
    cv::Mat pcl_mat (image_height, image_width, CV_32FC3, cv::Scalar(0));

    bool img_available = false;
    cv::Mat img_mat (image_height, image_width,  CV_8UC3, cv::Scalar(0)); 
    
    ros::Publisher camera_pub = nh.advertise<std_msgs::Bool>(camera_ch_send, queue);
    ros::Publisher main_pub = nh.advertise<std_msgs::Float32MultiArray>(core_ch_send, queue);
    
    

    auto pcl_callback = [&] (const sensor_msgs::PointCloud2ConstPtr &point_cloud) {
        std::cout << "[Calibration] Recieved Point Cloud..." << std::endl;
        
        pcl_mat = DataTypeHandler::PointCloud2Mat(point_cloud);
        pcl_available = true;

        if (pcl_available && img_available) {
            display_calibration(img_mat, pcl_mat);
        }
    };
    
    auto img_callback = [&] (const sensor_msgs::ImageConstPtr &image) {
        std::cout << "[Calibration] Received Image..." << std::endl;

        img_mat = DataTypeHandler::Image2Mat(image);
        img_available = true;

        if (pcl_available && img_available) {
            display_calibration(img_mat, pcl_mat);
        }
    };

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_ch_rcve, queue, pcl_callback);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>(image_ch_rcve, queue, img_callback);

    //Send request for new data
    std::cout << "[Calibration] Requesting new Camera Data"; 
    std::cin.get();
    std_msgs::Bool reply;
    reply.data = true;
    camera_pub.publish(reply);

    ros::spin();

    return EXIT_SUCCESS;

}