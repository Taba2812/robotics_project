#ifndef TYPES
#define TYPES

#include <opencv2/imgproc.hpp>

namespace Types {
    struct RecognitionResult {
        cv::Size2i template_size;
        cv::Vec4f detection_rect;
    }; 
}

#endif