#ifndef PIPELINE
#define PIPELINE

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "settings.h"

namespace pipeline {

    void removeNoise(cv::Mat src, cv::Mat dst);
    void generateMask(cv::Mat src, cv::Mat mask);
    void maskMat(cv::Mat src, cv::Mat masked, cv::Mat mask);

}

#endif