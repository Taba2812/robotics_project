#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "libraries/settings.h"
#include "libraries/display_utility.h"
#include "libraries/pipeline.h"

//VARIABLES -----------------

int frameNum = -1;

int erosion_size = 3;
int dilation_size = 5;

//---------------------------


int main () {

    //Get Test Image
    std::string url = setting::location + "blocks_group2_48.jpg";

    cv::Mat og_image = cv::imread(url, cv::IMREAD_COLOR);
    cv::Mat src_image(og_image, setting::getCropRect(og_image));

    if(src_image.empty()){
        exit(EXIT_FAILURE);
    }

    cv::Size src_size = cv::Size((int)src_image.cols, (int)src_image.rows);

    //Setup views
    utility::createWindows(src_size, 1);
    utility::addTrackbars();

    //Main Loop
    while (true) {
        frameNum += 1;

        std::cout << "Frame: #" << frameNum << std::endl;

        //PIPELINE--------------------
        cv::Mat blurred;
        pipeline::removeNoise(src_image, blurred);

        //Range Operation
        cv::Mat mask;
        pipeline::generateMask(blurred, &mask);

        //Masking Operation
        cv::Mat result;
        pipeline::maskMat(blurred, result, mask);
        //--------------------

        std::list<std::string> titles = {"Original", "Blurred", "Mask", "Result"};
        std::list<cv::Mat> mats = {src_image, blurred, mask, result};
        utility::showWindows(mats, titles);

        int c = cv::waitKey(10);
        if (c == 'k')
            break;
    }

    return EXIT_SUCCESS;
}