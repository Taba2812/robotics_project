#include "settings.h"

cv::Rect setting::getCropRect(cv::Mat mat) {
        return cv::Rect (mat.cols * (LEFT_CROP  / 100.0), 
                         mat.rows * (TOP_CROP   / 100.0), 
                         mat.cols * (RIGHT_CROP / 100.0), 
                         mat.rows * (BOT_CROP   / 100.0)); 
}

std::vector<setting::Boundry> setting::lookup_colors() {
        return {{{170,230,185},{108,125,66 }},  //Blocchetti Blu
                {{218,147,111},{140,51 ,50 }},  //Blocchetti Viola
                {{40 ,255,255},{0  ,130,115}}}; //Blocchetti Rossi
}