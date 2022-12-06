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

    int hue = 0;
    int saturation = 0;
    int brightness = 0;
    int step = 0;

    int erosion_size = 0;
    int dilation_size = 0;

    Size refS = Size((int) webcam_stream.get(CAP_PROP_FRAME_WIDTH),
                     (int) webcam_stream.get(CAP_PROP_FRAME_HEIGHT));

    namedWindow("Webcam Source", WINDOW_KEEPRATIO);
    createTrackbar("Hue", "Webcam Source", &hue, 360);
    createTrackbar("Saturation", "Webcam Source", &saturation, 255);
    createTrackbar("Brightness", "Webcam Source", &brightness, 255);
    createTrackbar("Step", "Webcam Source", &step, 50);
    createTrackbar("Erosion", "Webcam Source", &erosion_size, 50);
    createTrackbar("Dilation", "Webcam Source", &dilation_size, 50);


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

        //Range Operation
        Mat HSVCamera, HSVRange;
        cvtColor(frameReference, HSVCamera, COLOR_BGR2HSV);
        inRange(HSVCamera, Scalar(hue - step, saturation - step, brightness - step), Scalar(hue + step, saturation + step, brightness + step), HSVRange);

        //Mask Correction
        Mat erosion;
        Mat erosion_kernel = getStructuringElement( MORPH_CROSS, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
        erode(HSVRange, erosion, erosion_kernel);

        Mat dilation;
        Mat dilation_kernel = getStructuringElement( MORPH_CROSS, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
        dilate(HSVRange, dilation, dilation_kernel);

        //Masking Operation
        Mat Masked;
        bitwise_and(frameReference, frameReference, Masked, HSVRange);


        imshow("Webcam Source", Masked);
        imshow("Webcam HSV Range", HSVRange);
        imshow("Erosion", erosion);
        imshow("Dilation", dilation);


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
