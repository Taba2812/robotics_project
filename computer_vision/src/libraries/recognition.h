#ifndef RECOGNITION
#define RECOGNITION

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <list>
#include <iostream>

#include "settings.h"

namespace recognition {

    #define DATASET_PATH "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/blocks_dataset"

    //Change to 0 to skip detection of those blocks or colors

    void runRecognition(cv::InputOutputArray img);
    void setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil);
    void setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer);
    void getImagesWithRightColors(std::list<cv::Mat> *buffer, std::string block);
    void getImages(std::list<cv::Mat> *buffer, std::string block, std::string color);
    void detection(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer, cv::InputOutputArray img, std::vector<cv::Vec4f> position);

    void drawResults(cv::InputOutputArray img, std::vector<cv::Vec4f> position);
}

#endif