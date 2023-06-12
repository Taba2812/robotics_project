#ifndef DETECTION
#define DETECTION

#include <opencv2/core.hpp>
#include "settings.h"
#include "pipeline.h"
#include "recognition.h"
#include "location_handler.h"
#include "data_type_handler.h"

namespace Detection {

    struct DetectionResults {
        cv::Vec3f position;
        cv::Mat image;
    };

    DetectionResults detectBlocks(cv::Mat pcl, cv::Mat Png);
    DetectionResults detectGripper(cv::Mat pcl, cv::Mat png);
    DetectionResults Detect(cv::Mat pcl, cv::Mat png);

}

#endif