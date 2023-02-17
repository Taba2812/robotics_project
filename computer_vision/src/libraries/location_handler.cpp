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

cv::Vec3f LocationHandler::extrapolateDetectionPosition(cv::Vec4f selected, cv::InputArray point_cloud_array) {
    //Determine if a point is inside the selection
        //Would probably be better to get all the points in a separate array but then I would still need to iterate trough them
    //Add the thing to the karis average
        //This needs to be done for each of the components like x,y,z 
    //return the result of the karis average
}