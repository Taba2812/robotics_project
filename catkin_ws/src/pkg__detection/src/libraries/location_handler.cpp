#include "location_handler.h"

cv::Vec4f LocationHandler::selectDetection(std::vector<cv::Vec4f> detections, int width, int height) {
    if (detections.empty()) {return cv::Vec4f(0.0f, 0.0f, 0.0f, 0.0f);}
    
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
    cv::Mat matrice = point_cloud_array.getMat();
    cv::Vec4f midpoint = matrice.at<cv::Vec4f>(rect.center);
    float midpoint_distance = midpoint[3];

    //AVERAGE STUFF
    float weight_tot = 0; 
    float value_x_tot = 0;
    float value_y_tot = 0;
    float value_z_tot = 0;

    for (int r = 0; r < point_cloud_array.rows(); r++) {
        for (int c = 0; c < point_cloud_array.cols(); c++) {
            
            if (!cv::pointPolygonTest(contour, cv::Point2f((float)c,(float)r), false)) {continue;}

            cv::Point2i location(c,r);
            cv::Vec4f point = matrice.at<cv::Vec4f>(location);

            //Invalid inputs are set to 0.0f or close
            if (point[3] < 0.01f) {continue;} 
            if (point[3] >= 2.0f)
                std::cout << "Point Distance: " << point[3] << std::endl;

            float abs_dist_form_midpoint = abs(midpoint_distance - point[3]);
            float weight = 1 / (1 + abs_dist_form_midpoint); //Karis weight
            
            weight_tot += weight;
            value_x_tot += weight * point[0];
            value_y_tot += weight * point[1];
            value_z_tot += weight * point[2];
        }
    }

    float final_x = value_x_tot / weight_tot;
    float final_y = value_y_tot / weight_tot;
    float final_z = value_z_tot / weight_tot;

    return cv::Vec3f(final_x, final_y, final_z);
}

cv::Vec3f LocationHandler::extrapolateDetectionPosition(cv::Mat img, cv::Vec4f selected, cv::InputArray point_cloud_array, cv::Size2i template_size) {

    cv::RotatedRect rect = cv::RotatedRect (cv::Point2f(selected[0], selected[1]), 
                                            cv::Size2f(template_size.width * selected[2], template_size.height * selected[2]), 
                                            selected[3]);
    cv::Point2f corners[4];
    rect.points(corners);

    std::vector<cv::Point2f> contour;
    for (int v = 0; v < 4; v++) {
        contour.push_back(corners[v]);
    }

    //MIDPOINT STUFF
    cv::Mat matrice = point_cloud_array.getMat();
    cv::Vec4f midpoint = matrice.at<cv::Vec4f>(rect.center);
    float midpoint_distance = midpoint[3];

    //AVERAGE STUFF
    float weight_tot = 0; 
    float value_x_tot = 0;
    float value_y_tot = 0;
    float value_z_tot = 0;

    if (point_cloud_array.rows() != img.rows || point_cloud_array.cols() != img.cols) {return cv::Vec3f(0, 0, 0);}
    std::cout << "Rows: " << img.rows << " Cols: " << img.cols << std::endl;

    int i = 0;
    for (int r = 0; r < point_cloud_array.rows(); r++) {
        for (int c = 0; c < point_cloud_array.cols(); c++) {
            
            if (cv::pointPolygonTest(contour, cv::Point2f((float)c,(float)r), false) == 1) {
                cv::Point2i location(c,r);
                //cv::Vec4f point = matrice.at<cv::Vec4f>(location);
            
                //Invalid inputs are set to 0.0f or close
                //if (point[3] < 0.01f) {continue;}
                img.at<cv::Vec3i>(location) = {150,150,150};
                
                i++;
            }

            
        }
    }

    std::cout << "Total Pixels: " << img.rows * img.cols << " Eligible pixels: " << i << std::endl;

    return cv::Vec3f(0, 0, 0);
}