#ifndef LOC_HANDLER
#define LOC_HANDLER

#include <opencv2/imgproc.hpp>

namespace LocationHandler {

    cv::Vec4f selectDetection(std::vector<cv::Vec4f> detections, int width, int height);

}

#endif