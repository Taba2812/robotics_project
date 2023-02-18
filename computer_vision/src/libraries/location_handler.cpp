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

    cv::RotatedRect rect = cv::RotatedRect (cv::Point2f(selected[0], selected[1]), 
                                            cv::Size2f(template_size.width * selected[2], template_size.height * selected[2]), 
                                            selected[3]);
    cv::Point2f corners[4];
    rect.points(corners);

    //Converting array to a vector
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<cv::Point2f> contour(corners, lastItemPointer);

    //MIDPOINT STUFF
    int midpoint_x = template_size.width / 2;
    int midpoint_y = template_size.height / 2;
    cv::Point2i center(midpoint_y, midpoint_x);
    cv::Mat matrice = point_cloud_array.getMat();
    cv::Vec4f midpoint = matrice.at<cv::Vec4f>(center);
    float midpoint_distance = midpoint[3];

    //AVERAGE STUFF
    float weight_tot = 0; 
    float value_x_tot = 0;
    float value_y_tot = 0;
    float value_z_tot = 0;

    for (int r = 0; r < point_cloud_array.rows(); r++) {
        for (int c = 0; c < point_cloud_array.cols(); c++) {
            cv::Point2i location(r,c);
            cv::Vec4f point = matrice.at<cv::Vec4f>(location);

            std::cout << "x: " << point[0] << " y: " << point[1] << " z: " << point[2] << " d: " << point[3] << std::endl;
            
            if (!cv::pointPolygonTest(contour, cv::Point2f((float)c,(float)r), false)) {continue;}

            /*
            cv::Point2i location(r,c);
            cv::Vec4f point = matrice.at<cv::Vec4f>(location);
            float abs_dist_form_midpoint = abs(midpoint_distance - point[3]);

            float weight = 1 / (1 + abs_dist_form_midpoint); //Karis weight
            
            weight_tot += weight;
            value_x_tot += weight * point[0];
            value_y_tot += weight * point[1];
            value_z_tot += weight * point[2];
            */
        }
    }
    /*
    float final_x = value_x_tot / weight_tot;
    float final_y = value_y_tot / weight_tot;
    float final_z = value_z_tot / weight_tot;
    */
    return cv::Vec3f(0, 0, 0);
}