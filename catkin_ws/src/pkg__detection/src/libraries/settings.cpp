#include "settings.h"

cv::Rect setting::Container::getCropRect() {
        return cv::Rect (this->LEFT_CROP, 
                         this->TOP_CROP, 
                         this->RIGHT_CROP, 
                         this->BOT_CROP); 
}

std::vector<setting::Boundry> setting::Container::lookup_colors() {
        return {this->boundry};
}

setting::Container::Container() {
        this->X1_Y1_Z2 = 0;
        this->X1_Y2_Z1 = 0;
        this->X1_Y2_Z2 = 0;
        this->X1_Y2_Z2_CHAMFER = 0;
        this->X1_Y2_Z2_TWINFILLET = 0;
        this->X1_Y3_Z2 = 0;
        this->X1_Y3_Z2_FILLET = 0;
        this->X1_Y4_Z1 = 0;
        this->X1_Y4_Z2 = 0;
        this->X2_Y2_Z2 = 0;
        this->X2_Y2_Z2_FILLET = 0;

        this->BLUE = 0;
        this->ORANGE = 0;
        this->YELLOW = 0;
        this->RED = 0;
        this->VIOLET = 0;

        this->TOP_CROP = 0;
        this->BOT_CROP = 0;
        this->LEFT_CROP = 0;
        this->RIGHT_CROP = 0;

        this->EROSION_SIZE = 0;
        this->DILATION_SIZE = 0;

        this->boundry.lower.hue = 0;
        this->boundry.lower.saturation = 0;
        this->boundry.lower.brightness = 0;
        this->boundry.upper.hue = 0;
        this->boundry.upper.saturation = 0;
        this->boundry.upper.brightness = 0;

        this->guil.min_dist = 0;
        this->guil.levels = 0;
        this->guil.dp = 0;
        this->guil.max_buffer_size = 0;
        this->guil.min_angle = 0;
        this->guil.max_angle = 0;
        this->guil.step_angle = 0;
        this->guil.thresh_angle = 0;
        this->guil.min_scale = 0;
        this->guil.max_scale = 0;
        this->guil.step_scale = 0;
        this->guil.thresh_scale = 0;
        this->guil.thersh_pos = 0;
        this->guil.canny_thresh_low = 0;
        this->guil.canny_thresh_high = 0;

        this->intersection_threshold = 0;
        this->DATASET_PATH = "";
        
}

void setting::Container::SetParameters(ros::NodeHandle nh) {
        nh.getParam("X1_Y1_Z2",this->X1_Y1_Z2);
        nh.getParam("X1_Y2_Z1",this->X1_Y2_Z1);
        nh.getParam("X1_Y2_Z2",this->X1_Y2_Z2);
        nh.getParam("X1_Y2_Z2_CHAMFER",this->X1_Y2_Z2_CHAMFER);
        nh.getParam("X1_Y2_Z2_TWINFILLET",this->X1_Y2_Z2_TWINFILLET);
        nh.getParam("X1_Y3_Z2",this->X1_Y3_Z2);
        nh.getParam("X1_Y3_Z2_FILLET",this->X1_Y3_Z2_FILLET);
        nh.getParam("X1_Y4_Z1",this->X1_Y4_Z1);
        nh.getParam("X1_Y4_Z2",this->X1_Y4_Z2);
        nh.getParam("X2_Y2_Z2",this->X2_Y2_Z2);
        nh.getParam("X2_Y2_Z2_FILLET",this->X2_Y2_Z2_FILLET);

        nh.getParam("BLUE",this->BLUE);
        nh.getParam("ORANGE",this->ORANGE);
        nh.getParam("YELLOW",this->YELLOW);
        nh.getParam("RED",this->RED);
        nh.getParam("VIOLET",this->VIOLET);

        nh.getParam("LEFT_CROP",this->LEFT_CROP);
        nh.getParam("RIGHT_CROP",this->RIGHT_CROP);
        nh.getParam("TOP_CROP",this->TOP_CROP);
        nh.getParam("BOT_CROP",this->BOT_CROP);

        nh.getParam("EROSION_SIZE",this->EROSION_SIZE);
        nh.getParam("DILATION_SIZE",this->DILATION_SIZE);

        nh.getParam("LOWER_HUE",this->boundry.lower.hue);
        nh.getParam("LOWER_SATURATION",this->boundry.lower.saturation);
        nh.getParam("LOWER_BRIGHTNESS",this->boundry.lower.brightness);
        nh.getParam("HIGHER_HUE",this->boundry.upper.hue);
        nh.getParam("HIGHER_SATURATION",this->boundry.upper.saturation);
        nh.getParam("HIGHER_BRIGHTNESS",this->boundry.upper.brightness);

        nh.getParam("GUIL_MIN_DIST",this->guil.min_dist);
        nh.getParam("GUIL_LEVELS",this->guil.levels);
        nh.getParam("GUIL_DP",this->guil.dp);
        nh.getParam("GUIL_MAX_BUFFER_SIZE",this->guil.max_buffer_size);
        nh.getParam("GUIL_MIN_ANGLE",this->guil.min_angle);
        nh.getParam("GUIL_MAX_ANGLE",this->guil.max_angle);
        nh.getParam("GUIL_STEP_ANGLE",this->guil.step_angle);
        nh.getParam("GUIL_THRESH_ANGLE",this->guil.thresh_angle);
        nh.getParam("GUIL_MIN_SCALE",this->guil.min_scale);
        nh.getParam("GUIL_MAX_SCALE",this->guil.max_scale);
        nh.getParam("GUIL_STEP_SCALE",this->guil.step_scale);
        nh.getParam("GUIL_THRESH_SCALE",this->guil.thresh_scale);
        nh.getParam("GUIL_THRESH_POS",this->guil.thersh_pos);
        nh.getParam("GUIL_CANNY_THRESH_LOW",this->guil.canny_thresh_low);
        nh.getParam("GUIL_CANNY_THRESH_HIGH",this->guil.canny_thresh_high);

        nh.getParam("INTERSECTION_TRESHOLD", this->intersection_threshold);
        nh.getParam("BLOCK_DATASET_PATH", this->DATASET_PATH);
}