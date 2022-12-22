#ifndef PIPELINE
#define PIPELINE

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace pipeline {

    void removeNoise(cv::InputArray src, cv::OutputArray dst);
    void generateMask(cv::Mat src, cv::Mat *mask);
    void maskMat(cv::InputArray src, cv::OutputArray masked, cv::InputArray mask);

}

#endif