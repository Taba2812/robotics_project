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
    std::string url = setting::location + "blocks_group2_48.jpg";
    //Get Test Point Cloud
    cv::Mat point_cloud;
    TempFileHandler::LoadMatBinary("", point_cloud);

    cv::Mat og_image = cv::imread(url, cv::IMREAD_COLOR);
    cv::Mat src_image(og_image, setting::getCropRect(og_image));

    if(src_image.empty()){
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
        pipeline::maskMat(blurred, result, mask);

        //OBJECT RECOGNITION -------------------------------------------
        recognition::runRecognition(result);

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