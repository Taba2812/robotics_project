#ifndef RECOGNITION
#define RECOGNITION

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace recognition {

    #define DATASET_PATH "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/blocks_dataset"

    //Change to 0 to skip detection of those blocks or colors

    void runRecognition();
    void setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil);
    void setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil);

    void drawResults();
}

#endif