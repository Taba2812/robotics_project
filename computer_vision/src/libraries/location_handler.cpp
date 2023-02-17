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