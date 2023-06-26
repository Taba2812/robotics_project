#ifndef LOC_HANDLER
#define LOC_HANDLER

#include <opencv2/imgproc.hpp>
#include <iostream>

#include "util_types.h"

typedef cv::Point3_<uint8_t> ui8_Pixel;
typedef cv::Point3_<float> f32_Pixel;

namespace LocationHandler {

    Types::RecognitionResult selectDetection(std::vector<Types::RecognitionResult> detections, int width, int height);
    cv::Vec3f extrapolateDetectionPosition(cv::Mat img, Types::RecognitionResult selected, cv::Mat point_cloud_array);
    std::vector<cv::Point2i> RRect2Contour(Types::RecognitionResult selected);
}   

#endif