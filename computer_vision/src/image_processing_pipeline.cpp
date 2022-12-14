#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "settings.h"
#include "display_utility.h"

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
        printf(url.c_str());
        printf(" Error opening image\n");
        return EXIT_FAILURE;
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
        //Bilateral Blur on HSV Image   
        cv::Mat blurred;
        cv::bilateralFilter(src_image, blurred, 9, 75, 75);

        //Range Operation
        cv::Mat HSVCamera, HSVRange;
        cv::cvtColor(blurred, HSVCamera, cv::COLOR_BGR2HSV);

        cv::Mat dilation, closing, erosion, opening;
        cv::Mat mask((int)src_image.rows, (int)src_image.cols, CV_8UC1, cv::Scalar(0));

        //Mask for all colors that we are looking for then mix them
        for (setting::Boundry bound : setting::lookup_colors) {
            cv::inRange(HSVCamera, cv::Scalar(bound.lower.hue, bound.lower.saturation, bound.lower.brightness), 
                                   cv::Scalar(bound.upper.hue, bound.upper.saturation, bound.upper.brightness), HSVRange);

            //Mask Correction
            cv::Mat erosion_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
            cv::Mat dilation_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );

            cv::dilate(HSVRange, dilation, dilation_kernel);
            cv::erode(dilation, closing, erosion_kernel);

            cv::erode(closing, erosion, erosion_kernel);
            cv::dilate(erosion, opening, dilation_kernel);

            cv::bitwise_or(mask, opening, mask);
        }

        //Masking Operation
        cv::Mat Masked;
        cv::bitwise_and(src_image, src_image, Masked, mask);
        //--------------------

        std::list<cv::Mat> mats = {src_image, HSVCamera, HSVRange, opening, Masked};
        utility::showWindows(mats);

        int c = cv::waitKey(10);
        if (c == 'k')
            break;
    }

    return EXIT_SUCCESS;
}