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
    #define X1_Y2_Z1            1
    #define X1_Y2_Z2            1
    #define X1_Y2_Z2_CHAMFER    1
    #define X1_Y2_Z2_TWINFILLET 1
    #define X1_Y3_Z2            1
    #define X1_Y3_Z2_FILLET     1
    #define X1_Y4_Z1            1
    #define X1_Y4_Z2            1
    #define X2_Y2_Z2            1
    #define X2_Y2_Z2_FILLET     1

    #define BLUE   1
    #define ORANGE 1
    #define YELLOW 1
    #define RED    1
    #define VIOLET 1


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