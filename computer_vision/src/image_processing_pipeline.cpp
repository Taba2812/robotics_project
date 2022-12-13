#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::list<std::string> views = {"Original", "HSV", "Range", "Mask", "Masked"};
const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/official_placeholders/MultipleBlocks/";

struct HSV_Colorspace {
    int hue;
    int saturation;
    int brightness;
};

struct Boundry {
    HSV_Colorspace upper;
    HSV_Colorspace lower;
};

//VARIABLES -----------------

int top_crop = 15;
int bot_crop = 65;
int left_crop = 35;
int right_crop = 40;

int picture_index = 1;

int frameNum = -1;
int edge_treshold = 0;
int mask_treshold = 0;

int erosion_size = 3;
int dilation_size = 5;

std::vector<Boundry> lookup_colors = {{{170,230,185},{108,125,66 }},  //Blocchetti Blu
                                      {{218,147,111},{140,51 ,50 }},  //Blocchetti Viola
                                      {{40 ,255,255},{0  ,130,115}}}; //Blocchetti Rossi

//---------------------------

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

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
    cv::createTrackbar("X", *(views.begin()), &left_crop, 100);
    cv::createTrackbar("Y", *(views.begin()), &bot_crop, 100);
    cv::createTrackbar("Width", *(views.begin()), &right_crop, 100);
    cv::createTrackbar("Height", *(views.begin()), &top_crop, 100);
}

void saveImage(cv::Mat mat, std::string name) {
    std::string url = "../../output_renders/" + name;
    cv::imwrite(url, mat);
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
    std::string url = location + "blocks_group2_48.jpg";

    cv::Mat og_image = cv::imread(url, cv::IMREAD_COLOR);
    cv::Mat src_image(og_image, cv::Rect(og_image.cols * (left_crop / 100.0), 
                                         og_image.rows * (top_crop / 100.0), 
                                         og_image.cols * (right_crop / 100.0), 
                                         og_image.rows * (bot_crop / 100.0)));

    if(src_image.empty()){
        printf(url.c_str());
        printf(" Error opening image\n");
        return EXIT_FAILURE;
    }

    cv::Size src_size = cv::Size((int)src_image.cols, (int)src_image.rows);

    //Setup views
    createWindows(src_size, 1);
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

        cv::Mat dilation, closing, erosion, opening;
        cv::Mat mask((int)src_image.rows, (int)src_image.cols, CV_8UC1, cv::Scalar(0));

        //Mask for all colors that we are looking for then mix them
        for (Boundry bound : lookup_colors) {
            cv::inRange(HSVCamera, cv::Scalar(bound.lower.hue, bound.lower.saturation, bound.lower.brightness), 
                                   cv::Scalar(bound.upper.hue, bound.upper.saturation, bound.upper.brightness), HSVRange);

            //Mask Correction
            cv::Mat erosion_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
            cv::Mat dilation_kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );

            cv::dilate(HSVRange, dilation, dilation_kernel);
            cv::erode(dilation, closing, erosion_kernel);

            cv::erode(closing, erosion, erosion_kernel);
            cv::dilate(erosion, opening, dilation_kernel);

            cv::bitwise_or(mask, opening, mask);
        }

        //Masking Operation
        cv::Mat Masked;
        cv::bitwise_and(src_image, src_image, Masked, mask);
        //--------------------

        std::list<cv::Mat> mats = {src_image, HSVCamera, HSVRange, opening, Masked};

        showWindows(mats);

        int c = cv::waitKey(10);
        if (c == 'k')
            break;
        //EXIT condition
    }

    return EXIT_SUCCESS;
}