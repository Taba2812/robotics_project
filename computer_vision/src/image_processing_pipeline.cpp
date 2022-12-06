#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

int main()
{
    VideoCapture webcam_stream = VideoCapture(0);
    if (!webcam_stream.isOpened()) {
        std::cout << "Could not open webcam" << "0" << std::endl;
        return -1;
    }

    int frameNum = -1;
    int edge_treshold = 0;
    int mask_treshold = 0;

    Size refS = Size((int) webcam_stream.get(CAP_PROP_FRAME_WIDTH),
                     (int) webcam_stream.get(CAP_PROP_FRAME_HEIGHT));

    namedWindow("Webcam Source", WINDOW_KEEPRATIO);
    createTrackbar("Treshold", "Webcam Source", &edge_treshold, 100);
    createTrackbar("Mask Treshold", "Webcam Source", &mask_treshold, 255);

    namedWindow("Webcam Red", WINDOW_KEEPRATIO);
    namedWindow("Webcam Green", WINDOW_KEEPRATIO);
    namedWindow("Webcam Blue", WINDOW_KEEPRATIO);
    namedWindow("Red Mask", WINDOW_KEEPRATIO);
    namedWindow("Green Mask", WINDOW_KEEPRATIO);
    namedWindow("Blue Mask", WINDOW_KEEPRATIO);
    resizeWindow("Webcam Source", refS);
    resizeWindow("Webcam Red", refS/3);
    resizeWindow("Webcam Green", refS/3);
    resizeWindow("Webcam Blue", refS/3);
    resizeWindow("Red Mask", refS/3);
    resizeWindow("Green Mask", refS/3);
    resizeWindow("Blue Mask", refS/3);

    int bordo = 100;
    int barra = 75;

    moveWindow("Webcam Source", 0, 0);
    moveWindow("Webcam Red", refS.width + bordo, 0);
    moveWindow("Webcam Green", refS.width + bordo, refS.height/3 + bordo);
    moveWindow("Webcam Blue", refS.width + bordo, 2 * refS.height/3 + bordo + barra);

    moveWindow("Red Mask", 0, refS.height + bordo + barra);
    moveWindow("Green Mask", refS.width/3, refS.height + bordo + barra);
    moveWindow("Blue Mask", 2*refS.width/3, refS.height + bordo + barra);


    Mat frameReference, edgeDetected;
    std::vector<Mat> webcam_channels;
    std::vector<Mat> webcam_channels_masks;

    double psnrV;

    for (;;) {
        webcam_stream >> frameReference;
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



        imshow("Webcam Source", frameReference);
        imshow("Webcam Red", webcam_channels[2]);
        imshow("Webcam Green", webcam_channels[1]);
        imshow("Webcam Blue", webcam_channels[0]);
        imshow("Red Mask", webcam_channels_masks[2]);
        imshow("Green Mask", webcam_channels_masks[1]);
        imshow("Blue Mask", webcam_channels_masks[0]);

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
