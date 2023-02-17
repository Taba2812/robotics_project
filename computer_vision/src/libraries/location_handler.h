#ifndef LOC_HANDLER
#define LOC_HANDLER

#include <opencv2/imgproc.hpp>

namespace LocationHandler {

    cv::Vec4f selectDetection(std::vector<cv::Vec4f> detections, int width, int height);
    cv::Vec3f extrapolateDetectionPosition(cv::Vec4f selected, cv::InputArray point_cloud_array);

}

#endif