#ifndef SETTINGS
#define SETTINGS

#include <vector>
#include <opencv2/core.hpp>
#include <string>

namespace setting {
    

    #define RATIO 1000
    #define IMAGE_WIDTH 1920
    #define IMAGE_HEIGHT 1080


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

    class Container {
        public:
            //Blocks
            int X1_Y1_Z2;
            int X1_Y2_Z1;
            int X1_Y2_Z2;
            int X1_Y2_Z2_CHAMFER;
            int X1_Y2_Z2_TWINFILLET;
            int X1_Y3_Z2;
            int X1_Y3_Z2_FILLET;
            int X1_Y4_Z1;
            int X1_Y4_Z2;
            int X2_Y2_Z2;
            int X2_Y2_Z2_FILLET;

            //Colors
            int BLUE;
            int ORANGE;
            int YELLOW;
            int RED;
            int VIOLET;

            //Crops
            int TOP_CROP;
            int BOT_CROP;
            int LEFT_CROP;
            int RIGHT_CROP;

            //Image
            int EROSION_SIZE;
            int DILATION_SIZE;

            Container();
    };

    Container access;

    const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/complete_data_examples/";

    std::vector<Boundry> lookup_colors();

    cv::Rect getCropRect(cv::Mat mat);
    
}

#endif