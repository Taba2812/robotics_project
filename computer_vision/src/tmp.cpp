#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

const std::string location = "/home/dawwo/Documents/Repositories/robotics_project/computer_vision/images_database/testing_placeholders/";

int main()
{
    // Read from Webcam
    /*
    VideoCapture webcam_stream = VideoCapture(0);
    if (!webcam_stream.isOpened()) {
        std::cout << "Could not open webcam" << "0" << std::endl;
        return -1;
    }
    */

    int picture_index = 1;


    int frameNum = -1;
    int edge_treshold = 0;
    int mask_treshold = 0;

    int hue = 144;
    int saturation = 185;
    int brightness = 96;
    int step = 50;

    int erosion_size = 3;
    int dilation_size = 5;


    //Read from Images
    //Size refS = Size((int) webcam_stream.get(CAP_PROP_FRAME_WIDTH),
    //                 (int) webcam_stream.get(CAP_PROP_FRAME_HEIGHT));

    namedWindow("Webcam Source", WINDOW_KEEPRATIO);
    createTrackbar("Image", "Webcam Source", &picture_index, 9);
    createTrackbar("Hue", "Webcam Source", &hue, 360);
    createTrackbar("Saturation", "Webcam Source", &saturation, 255);
    createTrackbar("Brightness", "Webcam Source", &brightness, 255);
    createTrackbar("Step", "Webcam Source", &step, 100);
    createTrackbar("Erosion", "Webcam Source", &erosion_size, 50);
    createTrackbar("Dilation", "Webcam Source", &dilation_size, 50);

    std::string url = location + "Phone_Table_#2.jpg";

    Mat src_image = imread(url, IMREAD_COLOR);

    if(src_image.empty()){
        printf(url.c_str());
        printf(" Error opening image\n");
        return EXIT_FAILURE;
    }

    Size img_size = Size((int)src_image.cols, (int)src_image.rows);

    namedWindow("Webcam HSV Range", WINDOW_KEEPRATIO);
    resizeWindow("Webcam HSV Range", img_size/4);

    namedWindow("Image HSV", WINDOW_KEEPRATIO);
    resizeWindow("Image HSV", img_size/4);

    namedWindow("Open", WINDOW_KEEPRATIO);
    resizeWindow("Open", img_size/4);

    resizeWindow("Webcam Source", img_size/3);

    Mat frameReference, edgeDetected;
    std::vector<Mat> webcam_channels;
    std::vector<Mat> webcam_channels_masks;

    double psnrV;

    for (;;) {
        //webcam_stream >> frameReference;
        frameReference = src_image;
        split(frameReference, webcam_channels);
        split(frameReference, webcam_channels_masks);

        if (frameReference.empty()) {
            std::cout << "Game Over! " << std::endl;
            break;
        }
        
        frameNum += 1;

        std::cout << "Frame: #" << frameNum << std::endl;
        
        // Masking
        for (int i = 0; i < 3; i++) {
            threshold(webcam_channels[i], webcam_channels_masks[i], mask_treshold, 255, 0);
        }

        // Edge Detection
        blur(frameReference, edgeDetected, Size(3,3));
        Mat tmp;
        Canny(edgeDetected, tmp, edge_treshold, edge_treshold*3, 3);

        //Bilateral Blur on HSV Image
        Mat blurred;
        bilateralFilter(frameReference, blurred, 9, 75, 75);

        //Range Operation
        Mat HSVCamera, HSVRange;
        cvtColor(blurred, HSVCamera, COLOR_BGR2HSV);

        inRange(HSVCamera, Scalar(hue - step, saturation - step, brightness - step), Scalar(hue + step, saturation + step, brightness + step), HSVRange);

        //Mask Correction
        Mat erosion_kernel = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
        Mat dilation_kernel = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

        Mat dilation;
        dilate(HSVRange, dilation, dilation_kernel);
        Mat closing;
        erode(dilation, closing, erosion_kernel);

        Mat erosion;
        erode(closing, erosion, erosion_kernel);
        Mat opening;
        dilate(erosion, opening, dilation_kernel);

        //Masking Operation
        Mat Masked;
        bitwise_and(frameReference, frameReference, Masked, opening);


        imshow("Webcam Source", Masked);
        imshow("Webcam HSV Range", HSVRange);
        imshow("Image HSV", HSVCamera);
        imshow("Open", opening);


        int c = waitKey(10);
        if (c == 'k')
            break;
    }

    /*
    std::string image_path = samples::findFile("destiny_hunter.jpg");
    Mat img = imread(image_path, IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    imshow("Display window 2", img);
    int k = waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        imwrite("starry_night.png", img);
    }
    */
    return 0;
}

/*
PIPELINE ----------------------------

 - Separate RGB Channels

 - Treshold RGB Channels (When picking a single color need to do it up and down then mix)

 - Combine Masks
   - Opening And Closing to remove artifacts https://docs.opencv.org/4.x/d3/dbe/tutorial_opening_closing_hats.html

 - Apply Mask to Camera Input

 - Run Object Recognition

*/

/*
VALUES ------------------------------

 - Erosion 2
 - Dilation 7

 1 Kind of blocks
 - Hue 0
 - Saturation 209
 - Brightness 146
 - Step 50

 Other kind of block
 - Hue 144
 - Saturation 185
 - Brightness 96

*/
