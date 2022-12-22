#ifndef UTILITY
#define UTILITY

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <list>
#include <iostream>

namespace utility {

    std::string type2str(int type);
    void showWindows(std::list<cv::Mat> mats, std::list<std::string> titles);
    void createWindows(cv::Size src_size, int size_multiplier, std::list<std::string> titles);
    void addTrackbars();
    void saveImage(cv::Mat mat, std::string name);

}

#endif