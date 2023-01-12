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

    void runRecognition();
    void setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil);
    void setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil);
    void getImagesWithRightColors(std::list<cv::Mat> buffer, std::string block, int *counter);
    void getImages(std::list<cv::Mat> buffer, std::string block, std::string color, int *counter);
    void detection(cv::Ptr<cv::GeneralizedHoughGuil> guil);

    void drawResults();
}

#endif