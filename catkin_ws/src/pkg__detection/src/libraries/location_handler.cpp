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

std::vector<cv::Point2i> LocationHandler::RRect2Contour(cv::Vec4f data, cv::Size2i size) {
    std::vector<cv::Point2i> contour;

    cv::RotatedRect rect = cv::RotatedRect (cv::Point2f(data[0], data[1]), 
                                            cv::Size2f(size.width * data[2], size.height * data[2]), 
                                            data[3]);

    for (int i = 0; i < 4; i++) {
        std::cout << "Vector#" << i << " " << data[i] << std::endl;
    }
    std::cout << "TWidth: " << size.width << " THeight: " << size.height << std::endl;

    cv::Point2f vertices[4];
    rect.points(vertices);

    for (int i = 0; i < 4; i++) {
        cv::Point2i point = {(int)vertices[i].x, (int)vertices[i].y};
        contour.push_back(point);
    }

    return contour;
}

cv::Vec3f LocationHandler::extrapolateDetectionPosition(cv::Mat img, cv::Vec4f selected, cv::Mat point_cloud_array, cv::Size2i template_size) {

    
    std::vector<cv::Point2i> contour = LocationHandler::RRect2Contour(selected, template_size); 
    for (int i = 0; i < contour.size(); i++) {
        std::cout << "ContPt#" << i << " [" << contour[i].x << "," << contour[i].y << "]" << std::endl;
    }

    //MIDPOINT STUFF
    cv::Vec4f midpoint = point_cloud_array.at<cv::Vec4f>(selected[0], selected[1]);
    float midpoint_distance = midpoint[3];

    //AVERAGE STUFF
    float weight_tot = 0; 
    float value_x_tot = 0;
    float value_y_tot = 0;
    float value_z_tot = 0;

    if (point_cloud_array.rows != img.rows || point_cloud_array.cols != img.cols) {return cv::Vec3f(0, 0, 0);}
    std::cout << "Rows: " << img.rows << " Cols: " << img.cols << std::endl;

    int i = 0;
    auto pcl_lambda = [&] (f32_Pixel &pixel, const int *position) {
        int row = position[0];
        int col = position[1];
        if (!cv::pointPolygonTest(contour, cv::Point2f((float)col,(float)row), false)) {return;}

        float distance = sqrt(pow(pixel.x,2) + pow(pixel.y,2) + pow(pixel.z,2));
        if (distance < 0.01f) {return;}

        float abs_dist_form_midpoint = abs(midpoint_distance - distance);
        float weight = 1 / (1 + abs_dist_form_midpoint); //Karis weight

        weight_tot += weight;
        value_x_tot += weight * pixel.x;
        value_y_tot += weight * pixel.y;
        value_z_tot += weight * pixel.z;

        i++;
    };

    auto img_lambda = [&] (ui8_Pixel &pixel, const int *position) {
        int row = position[0];
        int col = position[1];
        if (cv::pointPolygonTest( contour, cv::Point2f((float)col, (float)row), false ) != 0) {return;}
        
        pixel.x = 150;
        pixel.y = 150;
        pixel.z = 150;
    };

    point_cloud_array.forEach<f32_Pixel>(pcl_lambda);
    img.forEach<ui8_Pixel>(img_lambda);

    float final_x = value_x_tot / weight_tot;
    float final_y = value_y_tot / weight_tot;
    float final_z = value_z_tot / weight_tot;
    
    std::cout << "Total Pixels: " << img.rows * img.cols << " Eligible pixels: " << i << std::endl;

    return cv::Vec3f(final_x, final_y, final_z);
}