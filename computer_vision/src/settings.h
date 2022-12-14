#ifndef SETTINGS
#define SETTINGS

#include <vector>
#include <opencv2/core.hpp>

namespace setting {
    //VALUES
    #define TOP_CROP 15
    #define BOT_CROP 65
    #define LEFT_CROP 35
    #define RIGHT_CROP 40

    #define EROSION_SIZE 3
    #define DILATION_SIZE 5

    //STRUCTS
    struct HSV_Colorspace {
        int hue;
        int saturation;
        int brightness;
    };

    struct Boundry {
        HSV_Colorspace upper;
        HSV_Colorspace lower;
    };

    const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/official_placeholders/MultipleBlocks/";

    std::vector<Boundry> lookup_colors = {{{170,230,185},{108,125,66 }},  //Blocchetti Blu
                                          {{218,147,111},{140,51 ,50 }},  //Blocchetti Viola
                                          {{40 ,255,255},{0  ,130,115}}}; //Blocchetti Rossi

    cv::Rect getCropRect(cv::Mat mat) {
        return cv::Rect (mat.cols * (LEFT_CROP  / 100.0), 
                         mat.rows * (TOP_CROP   / 100.0), 
                         mat.cols * (RIGHT_CROP / 100.0), 
                         mat.rows * (BOT_CROP   / 100.0)); 
    }
    
}

#endif