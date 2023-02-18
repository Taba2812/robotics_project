#include "location_handler.h"

cv::Vec4f LocationHandler::selectDetection(std::vector<cv::Vec4f> detections, int width, int height) {
    // Select the detection that is cloasest to the center
    int mid_x = (float)width / 2;
    int mid_y = (float)height / 2;

    int i = 0;
    int selected = 0;
    float dist = 0;
    for (cv::Vec4f det : detections) {

        float distance = sqrt(pow(abs(mid_x - det[0]),2) + pow(abs(mid_y - det[1]),2));

        if (i == 0) {
            dist = distance;
        } else {
            if (distance < dist) {
                dist = distance;
                selected = i;
            }
        }

        i++;
    }

    return detections[selected];
}

cv::Vec3f LocationHandler::extrapolateDetectionPosition(cv::Vec4f selected, cv::InputArray point_cloud_array, cv::Size2i template_size) {
    //Make the Vec4f into a Contour
    //Determine if a point is inside the selection
        //Would probably be better to get all the points in a separate array but then I would still need to iterate trough them
    //Add the thing to the karis average
        //This needs to be done for each of the components like x,y,z 
    //return the result of the karis average

    cv::RotatedRect rect = cv::RotatedRect (cv::Point2f(selected[0], selected[1]), 
                                            cv::Size2f(template_size.width * selected[2], template_size.height * selected[2]), 
                                            selected[3]);
    cv::Point2f corners[4];
    rect.points(corners);

    //Converting array to a vector
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<cv::Point2f> contour(corners, lastItemPointer);


    for (int r = 0; r < point_cloud_array.rows(); r++) {
        for (int c = 0; c < point_cloud_array.cols(); c++) {
            if (!cv::pointPolygonTest(contour, cv::Point2f((float)c,(float)r), false)) {continue;}
            //Add to the average

        }
    }
}