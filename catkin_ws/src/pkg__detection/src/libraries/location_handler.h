#ifndef LOC_HANDLER
#define LOC_HANDLER

#include <opencv2/imgproc.hpp>
#include <iostream>

typedef cv::Point3_<uint8_t> ui8_Pixel;
typedef cv::Point3_<float> f32_Pixel;

namespace LocationHandler {

    cv::Vec4f selectDetection(std::vector<cv::Vec4f> detections, int width, int height);
    cv::Vec3f extrapolateDetectionPosition(cv::Mat img, cv::Vec4f selected, cv::Mat point_cloud_array, cv::Size2i template_size);
    std::vector<cv::Point2i> RRect2Contour(cv::Vec4f data, cv::Size2i size);
}   

#endif