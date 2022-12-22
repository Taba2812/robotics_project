#include "display_utility.h"

std::string utility::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
};

void utility::createWindows(cv::Size src_size, int size_multiplier, std::list<std::string> titles) {
    int ratio = titles.size() / 3;

    cv::Size view_size = src_size / (ratio*size_multiplier);
    
    for (std::string s : titles) {
        cv::namedWindow(s, cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(s, view_size);
    }

}

void utility::addTrackbars() {
    //cv::createTrackbar("",*(views.begin()), &)
}

void utility::showWindows(std::list<cv::Mat> mats, std::list<std::string> titles) {
    std::list<std::string>::iterator it_views = titles.begin();
    std::list<cv::Mat>::iterator     it_mats  = mats.begin();

    for (;it_views != titles.end();) {
        cv::imshow(*it_views, *it_mats);

        it_views++;
        it_mats++;
    }
}

void utility::saveImage(cv::Mat mat, std::string name) {
    std::string url = "../../output_renders/" + name;
    cv::imwrite(url, mat);
}