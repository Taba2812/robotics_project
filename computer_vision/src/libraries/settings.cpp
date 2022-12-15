#include "settings.h"

cv::Rect setting::getCropRect(cv::Mat mat) {
        return cv::Rect (mat.cols * (LEFT_CROP  / 100.0), 
                         mat.rows * (TOP_CROP   / 100.0), 
                         mat.cols * (RIGHT_CROP / 100.0), 
                         mat.rows * (BOT_CROP   / 100.0)); 
}