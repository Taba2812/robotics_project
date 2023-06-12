#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

int main (int argc, char **argv) {
    ros::init(argc, argv, "DetectionDebug_Dummy");

    ros::NodeHandle handle;
    int RATIO;
    std::string topic;
    handle.getParam("Q_Size", RATIO);
    handle.getParam("Det2Deb_Img", topic);

    auto image_callback = [&] (const sensor_msgs::ImageConstPtr &result) {
        
        
        cv_bridge::CvImagePtr ImgPtr;
        
        try {
            ImgPtr = cv_bridge::toCvCopy(result, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        
        while (true) {
            cv::imshow("Detection Output", ImgPtr->image);
            int c = cv::waitKey(10);
            if (c == 'k') {   
                break;
            }
        }        
        
    };

    ros::Subscriber sub_img = handle.subscribe<sensor_msgs::Image>(topic, RATIO, image_callback);

    std::cout << "Spinning ..." << std::endl;
    ros::spin();
}