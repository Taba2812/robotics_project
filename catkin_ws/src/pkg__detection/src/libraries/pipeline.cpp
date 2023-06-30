#include "pipeline.h"
#include "settings.h"

void pipeline::removeNoise(cv::InputArray src, cv::OutputArray dst) {
    //Bilateral Blur on Image   
    cv::bilateralFilter(src, dst, 9, 75, 75);
}

void pipeline::generateMask(cv::Mat src, cv::Mat *mask) {
    cv::Mat HSVCamera, HSVRange;
    cv::cvtColor(src, HSVCamera, cv::COLOR_BGR2HSV);

    cv::Mat dilation, closing, erosion, opening;
    *mask = cv::Mat((int)src.rows, (int)src.cols, CV_8UC1, cv::Scalar(0));

    //Mask for all colors that we are looking for then mix them
    for (setting::Boundry bound : setting::access.lookup_colors()) {
        cv::inRange(HSVCamera, cv::Scalar(bound.lower.hue, bound.lower.saturation, bound.lower.brightness), 
                               cv::Scalar(bound.upper.hue, bound.upper.saturation, bound.upper.brightness), HSVRange);

        //Mask Correction
        cv::Mat erosion_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2* setting::access.EROSION_SIZE + 1, 2* setting::access.EROSION_SIZE +1 ), 
                                                                               cv::Point( setting::access.EROSION_SIZE, setting::access.EROSION_SIZE ) );
        
        cv::Mat dilation_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2* setting::access.DILATION_SIZE + 1, 2* setting::access.DILATION_SIZE +1 ), 
                                                                                cv::Point( setting::access.DILATION_SIZE, setting::access.DILATION_SIZE ) );

        cv::dilate(HSVRange, dilation, dilation_kernel);
        cv::erode(dilation, closing, erosion_kernel);

        cv::erode(closing, erosion, erosion_kernel);
        cv::dilate(erosion, opening, dilation_kernel);

        //cv::bitwise_or(*mask, opening, *mask);
        *mask = opening;
    }
}

void pipeline::maskMat(cv::InputArray src, cv::OutputArray masked, cv::InputArray mask) {
    cv::bitwise_and(src, src, masked, mask);
}