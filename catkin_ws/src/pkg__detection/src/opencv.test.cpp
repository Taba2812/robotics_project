#include "ros/ros.h"

#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

typedef cv::Point3_<uint8_t> Pixel;

#define OOTO 1

int main (int argc, char** argv) {

    ros::init(argc, argv, "OpenCV_Test");
    ros::NodeHandle nh;

    std::string file_path;
    nh.getParam("FILE_PATH", file_path);


    std::cout << file_path << std::endl;
#if OOTO
    
    cv::Mat img(400,400, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat contour_proxy = cv::Mat::zeros(img.size(), CV_8UC1);

    cv::RotatedRect rect = cv::RotatedRect(cv::Point2f(200, 200), cv::Size2f(100, 50),90);

    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        std::cout << "Vertex#" << i << " [" << vertices[i].x << "," << vertices[i].y << "]" << std::endl;
        cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
    } 

    std::vector<cv::Point2i> contour;

    for (int i = 0; i < 4; i++) {
        cv::Point2i point = {(int)vertices[i].x, (int)vertices[i].y};
        contour.push_back(point);
    }

    for (int i = 0; i < contour.size(); i++) {
        std::cout << "ContPt#" << i << " [" << contour[i].x << "," << contour[i].y << "]" << std::endl;
    }
    
    
    for (int r = 0; r < contour_proxy.rows; r++) {
        for (int c = 0; c < contour_proxy.cols; c++) {
            if (cv::pointPolygonTest( contour, cv::Point2f((float)c, (float)r), false ) == 1) {
                contour_proxy.at<u_int8_t>(r,c) = 255;
            }
        }
    }

    auto test_lamb = [&] (Pixel &pixel, const int *position) {
        int row = position[0];
        int col = position[1];
        if (cv::pointPolygonTest( contour, cv::Point2f((float)col, (float)row), false ) == 1) {
            pixel.x = 255;
        }
    };

    img.forEach<Pixel>(test_lamb);

    cv::imwrite(file_path, img);

#else 
    const int r = 100;
 cv::Mat src = cv::Mat::zeros( cv::Size( 4*r, 4*r ), CV_8UC3 );
 std::vector<cv::Point2f> vert(6);
 vert[0] = cv::Point( 3*r/2, static_cast<int>(1.34*r) );
 vert[1] = cv::Point( 1*r, 2*r );
 vert[2] = cv::Point( 3*r/2, static_cast<int>(2.866*r) );
 vert[3] = cv::Point( 5*r/2, static_cast<int>(2.866*r) );
 vert[4] = cv::Point( 3*r, 2*r );
 vert[5] = cv::Point( 5*r/2, static_cast<int>(1.34*r) );
 for( int i = 0; i < 6; i++ )
 {
    std::cout << "Vertex#" << i << " [" << vert[i].x << "," << vert[i].y << "]" << std::endl;
    cv::line( src, vert[i], vert[(i+1)%6], cv::Scalar( 255 ), 3 );
 }
 std::vector<std::vector<cv::Point>> contours;
 //cv::findContours( src, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
 std::vector<cv::Point> con;
 for (int i = 0; i < 6; i++) {
    con.push_back(vert[i]);
 }
 contours = {con};
std::cout << std::endl;
int point_size = contours[0].size();
cv::Mat raw_dist( src.size(), CV_32F );
 for (int i = 0; i < point_size; i++) {
    std::cout << "ContPt#" << i << " [" << contours[0][i].x << "," << contours[0][i].y << "]" << std::endl;
    raw_dist.at<float>(contours[0][i].y,contours[0][i].x) = 25*(i+1);
}

 
 for( int i = 0; i < src.rows; i++ )
 {
 for( int j = 0; j < src.cols; j++ )
 {
 raw_dist.at<float>(i,j) = (float)cv::pointPolygonTest( contours[0], cv::Point2f((float)j, (float)i), false ) * 200;
 }
 }

    cv::imwrite(file_path, raw_dist);

#endif
    return EXIT_SUCCESS;
}