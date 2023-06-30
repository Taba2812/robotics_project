#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#define DISPLAY 0

int main (int argc, char **argv) {
    ros::init(argc, argv, "DetectionDebug_Dummy");

    ros::NodeHandle handle;
    int RATIO;
    std::string topic, file_path;
    handle.getParam("Q_Size", RATIO);
    handle.getParam("Det2Deb_Img", topic);
    handle.getParam("OUT_PATH", file_path);

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &result) {
        
        
        cv_bridge::CvImagePtr ImgPtr;
        
        try {
            ImgPtr = cv_bridge::toCvCopy(result, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        #if DISPLAY
        while (true) {
            std::cout << "[Debug][Detection] Showing final result of detection image processing on screen..." << std::endl;
            cv::imshow("Detection Output", ImgPtr->image);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                break;
            }
        } 
        #else
            std::cout << "[Debug][Detection] Saving final result of detection image processing in .png file..." << std::endl;
            cv::imwrite(file_path, ImgPtr->image);
        #endif   
        
    };

    ros::Subscriber sub_img = handle.subscribe<sensor_msgs::Image>(topic, RATIO, image_callback);

    ros::spin();
}