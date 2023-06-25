#ifndef RECOGNITION
#define RECOGNITION

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <list>
#include <iostream>
#include <math.h>

#include "settings.h"

namespace recognition {

    std::vector<cv::Vec4f> runRecognition(cv::InputOutputArray img);
    void setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil);
    void setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer);
    void getImagesWithRightColors(std::list<cv::Mat> *buffer, std::string block);
    void getImages(std::list<cv::Mat> *buffer, std::string block, std::string color);
    std::vector<cv::Vec4f> detection(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer, cv::InputOutputArray img, std::vector<cv::Vec4f> position);

    void drawResults(cv::InputOutputArray img, std::vector<cv::Vec4f> position, std::list<cv::Mat> *buffer, std::vector<cv::Mat> pTemplate);
    void drawSelected(cv::Mat img, cv::Vec4f selection);
    void scrapOvelappingDetections(std::vector<cv::Vec4f> *detections, std::vector<cv::Mat> *pTemplate);
    void compareRotatedRects(std::vector<cv::Vec4f> *position, std::vector<cv::Vec4f>::iterator beginning, std::vector<cv::Vec4f>::iterator ending, cv::RotatedRect rect_to_compare, int area_to_compare, int height, int width);
    bool distanceCondition(cv::Vec4f first, cv::Size2i first_size, cv::Vec4f second, cv::Size2i second_size);
}

#endif