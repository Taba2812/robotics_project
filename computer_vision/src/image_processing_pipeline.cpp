#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::list<std::string> views = {"Original", "HSV", "Mask", "Masked"};
const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/official_placeholders/MultipleBlocks/";

struct HSV_Colorspace{
    int hue;
    int saturation;
    int brightness;
};

std::vector<HSV_Colorspace> lookup_colors = {{107,194,139}};

//VARIABLES -----------------

int picture_index = 1;

int frameNum = -1;
int edge_treshold = 0;
int mask_treshold = 0;

int hue = 107;
int saturation = 194;
int brightness = 139;
int step = 50;

int erosion_size = 3;
int dilation_size = 5;

//---------------------------

void createWindows(cv::Size src_size, int size_multiplier) {
    int ratio = views.size() / 3;

    cv::Size view_size = src_size / (ratio*size_multiplier);
    
    for (std::string s : views) {
        cv::namedWindow(s, cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(s, view_size);
    }

}

void addTrackbars() {
    //cv::createTrackbar("",*(views.begin()), &)
    cv::createTrackbar("Image", *(views.begin()), &picture_index, 9);
    cv::createTrackbar("Hue", *(views.begin()), &hue, 360);
    cv::createTrackbar("Saturation", *(views.begin()), &saturation, 255);
    cv::createTrackbar("Brightness", *(views.begin()), &brightness, 255);
    cv::createTrackbar("Step", *(views.begin()), &step, 100);
    cv::createTrackbar("Erosion", *(views.begin()), &erosion_size, 50);
    cv::createTrackbar("Dilation", *(views.begin()), &dilation_size, 50);
}

void showWindows(std::list<cv::Mat> mats) {
    std::list<std::string>::iterator it_views = views.begin();
    std::list<cv::Mat>::iterator     it_mats  = mats.begin();

    for (;it_views != views.end();) {
        cv::imshow(*it_views, *it_mats);

        it_views++;
        it_mats++;
    }
}

int main () {

    //Get Test Image
    std::string url = location + "blocks_group2_14.jpg";

    cv::Mat src_image = cv::imread(url, cv::IMREAD_COLOR);

    if(src_image.empty()){
        printf(url.c_str());
        printf(" Error opening image\n");
        return EXIT_FAILURE;
    }

    cv::Size src_size = cv::Size((int)src_image.cols, (int)src_image.rows);

    //Setup views
    createWindows(src_size, 2);
    addTrackbars();

    //Main Loop
    while (true) {

        frameNum += 1;

        std::cout << "Frame: #" << frameNum << std::endl;

        //PIPELINE--------------------
        //Bilateral Blur on HSV Image   
        cv::Mat blurred;
        cv::bilateralFilter(src_image, blurred, 9, 75, 75);

        //Range Operation
        cv::Mat HSVCamera, HSVRange;
        cv::cvtColor(blurred, HSVCamera, cv::COLOR_BGR2HSV);

        cv::Mat dilation, closing, erosion, opening, mask;
        //Mask for all colors that we are looking for then mix them
        //for (HSV_Colorspace pick : lookup_colors) {
            cv::inRange(HSVCamera, cv::Scalar(hue - step, saturation - step, brightness - step), cv::Scalar(hue + step, saturation + step, brightness + step), HSVRange);

            //Mask Correction
            cv::Mat erosion_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
            cv::Mat dilation_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );

            cv::dilate(HSVRange, dilation, dilation_kernel);
            cv::erode(dilation, closing, erosion_kernel);

            cv::erode(closing, erosion, erosion_kernel);
            cv::dilate(erosion, opening, dilation_kernel);

            //cv::bitwise_or(mask, opening, mask);
        //}

        //Masking Operation
        cv::Mat Masked;
        cv::bitwise_and(src_image, src_image, Masked, opening);
        //--------------------

        std::list<cv::Mat> mats = {src_image, HSVCamera, opening, Masked};

        showWindows(mats);

        int c = cv::waitKey(10);
        if (c == 'k')
            break;
        //EXIT condition
    }

    return EXIT_SUCCESS;
}