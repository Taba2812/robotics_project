#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <ros.h>

#include "libraries/settings.h"
#include "libraries/display_utility.h"
#include "libraries/pipeline.h"
#include "libraries/recognition.h"
#include "libraries/temp_file_handler.h"
#include "libraries/location_handler.h"
#include "libraries/detection.h"

//VARIABLES -----------------

int frameNum = -1;

int erosion_size = 3;
int dilation_size = 5;

//---------------------------


int main () {

    //SETUP ------------------------------------------------------------------
    /* ROS STUFF
        void get_state(const [ros_type]::ConstPtr& [name]){ ... }
        ros::Subscriber vision_sub = nh.subscribe("state", QUEUE_SIZE, get_state);
        ros::Publisher vision_pub = nh.advertise<ros_type>("block_position", QUEUE_SIZE);
    */
    std::list<std::string> titles = {"Result"};

    //Get Test Image
    std::string image_url = setting::location + "Example_Image_Color_SingleBlock.png";
    cv::Mat og_image = cv::imread(image_url, cv::IMREAD_COLOR);
    //Get Test Point Cloud
    cv::Mat point_cloud;
    std::string data_url = setting::location + "ExampleMatrix_SingleBlock";
    TempFileHandler::LoadMatBinary(data_url, point_cloud);

    //Crop Images
    cv::Mat src_image (og_image, setting::getCropRect(og_image));
    cv::Mat data_image(point_cloud, setting::getCropRect(point_cloud));

    if(src_image.empty() or data_image.empty()){
        exit(EXIT_FAILURE);
    }

    cv::Size src_size = cv::Size((int)src_image.cols, (int)src_image.rows);

    //Setup views
    utility::createWindows(src_size, 1, titles);
    utility::addTrackbars();

    //MAIN LOOP --------------------------------------------------------
    while (true) {
        frameNum += 1;

        std::cout << "Frame: #" << frameNum << " --------------------------------------------------------" << std::endl;

        //IMAGE PROCESSING PIPELINE-------------------------------------
        cv::Mat blurred;
        pipeline::removeNoise(src_image, blurred);

        //Range Operation
        cv::Mat mask;
        pipeline::generateMask(blurred, &mask);

        //Masking Operation
        cv::Mat result;
        cv::Mat pcd;
        pipeline::maskMat(blurred, result, mask);
        pipeline::maskMat(data_image, pcd, mask);

        //OBJECT RECOGNITION -------------------------------------------
        std::vector<cv::Vec4f> detections;
        detections = recognition::runRecognition(result);

        //CALCULATION OBJECT POSITION
        cv::Vec4f selected_block;
        selected_block = LocationHandler::selectDetection(detections, result.cols, result.rows);

        cv::Vec3f detection_position;
        detection_position = LocationHandler::extrapolateDetectionPosition(selected_block, pcd, pcd.size());

        std::cout << "Final Position | X: " << detection_position[0] << " Y: " << detection_position[1] << " Z: " << detection_position[2] << std::endl;

        //DISPLAYING PROCESS ON SCREEN ---------------------------------
        std::list<cv::Mat> mats = {result};
        utility::showWindows(mats, titles);

        int c = cv::waitKey(10);
        if (c == 'k') {
            utility::saveImage(result, "Immagine Template.jpg");
            break;
        }
    }

    return EXIT_SUCCESS;
}