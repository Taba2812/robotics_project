#include "libraries/temp_file_handler.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
//#include "ros/ros.h"

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

int main (int argc, char **argv) {

    //Open the file
    std::cout << "Starting Program" << std::endl;
    cv::Mat matrice;
    TempFileHandler::LoadMatBinary("/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/ExampleMatrix_SingleBlock", matrice);
    std::cout << "Got the file 2" << std::endl;
    std::cout << "Tipo della matrice: " << type2str(matrice.type()) << std::endl;
    cv::cvtColor(matrice, matrice, cv::COLOR_RGBA2RGB);
    //matrice = cv::imread("/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/PointCloud_Matrix_SingleBlocks.png", CV_32FC4);
    cv::imwrite("/home/dawwo/Documents/Repositories/robotics_project/catkin_ws/src/images_database/complete_data_examples/PointCloud_Matrix_SingleBlocks.png", matrice);

    while (true) {
        cv::imshow("Window", matrice);
        int c = cv::waitKey(10);
        if (c == 'k') {
            break;
        }
    } 

    return 0;
}