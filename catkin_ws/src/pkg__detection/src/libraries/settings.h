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

    #define EROSION_SIZE 2
    #define DILATION_SIZE 5

    #define RATIO 1000
    #define IMAGE_WIDTH 1920
    #define IMAGE_HEIGHT 1080

    //IMGAGE RECOGNITION SETTINGS
    #define X1_Y1_Z2            0
    #define X1_Y2_Z1            0
    #define X1_Y2_Z2            0
    #define X1_Y2_Z2_CHAMFER    0
    #define X1_Y2_Z2_TWINFILLET 0
    #define X1_Y3_Z2            0
    #define X1_Y3_Z2_FILLET     0
    #define X1_Y4_Z1            0
    #define X1_Y4_Z2            0
    #define X2_Y2_Z2            1
    #define X2_Y2_Z2_FILLET     0

    #define BLUE   1
    #define ORANGE 1
    #define YELLOW 1
    #define RED    1
    #define VIOLET 1

    //IMAGE OVERLAP
    #define AREA_INTERSECTION_TRESHOLD 0.5

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

    const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/complete_data_examples/";

    std::vector<Boundry> lookup_colors();

    cv::Rect getCropRect(cv::Mat mat);
    
}

#endif