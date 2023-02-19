#ifndef DETECTION
#define DETECTION

#include <opencv2/core.hpp>
#include "settings.h"
#include "pipeline.h"
#include "recognition.h"
#include "location_handler.h"

namespace Detection {

    cv::Vec3f detectBlocks(cv::Mat pcl, cv::Mat Png);
    cv::Vec3f detectGripper(cv::Mat pcl, cv::Mat png);
    cv::Vec3f Detect(cv::Mat pcl, cv::Mat png);

}

#endif