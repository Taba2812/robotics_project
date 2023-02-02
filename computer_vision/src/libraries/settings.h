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

    //IMGAGE RECOGNITION SETTINGS
    #define X1_Y1_Z2            1
    #define X1_Y2_Z1            0
    #define X1_Y2_Z2            0
    #define X1_Y2_Z2_CHAMFER    0
    #define X1_Y2_Z2_TWINFILLET 0
    #define X1_Y3_Z2            0
    #define X1_Y3_Z2_FILLET     0
    #define X1_Y4_Z1            0
    #define X1_Y4_Z2            0
    #define X2_Y2_Z2            0
    #define X2_Y2_Z2_FILLET     0

    #define BLUE   0
    #define ORANGE 1
    #define YELLOW 0
    #define RED    0
    #define VIOLET 0

    //IMAGE OVERLAP
    #define AREA_INTERSECTION_TRESHOLD 0.7

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

    std::vector<Boundry> lookup_colors();

    cv::Rect getCropRect(cv::Mat mat);
    
}

#endif