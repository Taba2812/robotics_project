#ifndef UTILITY
#define UTILITY

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <list>

namespace utility {

    std::list<std::string> views = {"Original", "HSV", "Range", "Mask", "Masked"};

    std::string type2str(int type);
    void showWindows(std::list<cv::Mat> mats);
    void createWindows(cv::Size src_size, int size_multiplier);
    void addTrackbars();
    void saveImage(cv::Mat mat, std::string name);

}

#endif