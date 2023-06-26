#include "location_handler.h"

Types::RecognitionResult LocationHandler::selectDetection(std::vector<Types::RecognitionResult> detections, int width, int height) {
    if (detections.empty()) {return Types::RecognitionResult();}
    
    // Select the detection that is cloasest to the center
    int mid_x = (float)width / 2;
    int mid_y = (float)height / 2;

    int i = 0;
    int selected = 0;
    float dist = 0;
    for (Types::RecognitionResult det : detections) {

        float distance = sqrt(pow(abs(mid_x - det.detection_rect[0]),2) + pow(abs(mid_y - det.detection_rect[1]),2));

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

std::vector<cv::Point2i> LocationHandler::RRect2Contour(Types::RecognitionResult selected) {
    std::vector<cv::Point2i> contour;

    cv::RotatedRect rect = cv::RotatedRect (cv::Point2f(selected.detection_rect[0], selected.detection_rect[1]), 
                                            cv::Size2f(selected.template_size.width * selected.detection_rect[2], selected.template_size.height * selected.detection_rect[2]), 
                                            selected.detection_rect[3]);

    for (int i = 0; i < 4; i++) {
        std::cout << "Vector#" << i << " " << selected.detection_rect[i] << std::endl;
    }
    std::cout << "TWidth: " << selected.template_size.width << " THeight: " << selected.template_size.height << std::endl;

    cv::Point2f vertices[4];
    rect.points(vertices);

    for (int i = 0; i < 4; i++) {
        cv::Point2i point = {(int)vertices[i].x, (int)vertices[i].y};
        contour.push_back(point);
    }

    return contour;
}

cv::Vec3f LocationHandler::extrapolateDetectionPosition(cv::Mat img, Types::RecognitionResult selected, cv::Mat point_cloud_array) {

    
    std::vector<cv::Point2i> contour = LocationHandler::RRect2Contour(selected); 
    for (int i = 0; i < contour.size(); i++) {
        std::cout << "ContPt#" << i << " [" << contour[i].x << "," << contour[i].y << "]" << std::endl;
    }

    //MIDPOINT STUFF
    cv::Vec4f midpoint = point_cloud_array.at<cv::Vec4f>(selected.detection_rect[0], selected.detection_rect[1]);
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
        if (cv::pointPolygonTest(contour, cv::Point2f((float)col,(float)row), false) != 1) {return;}

        float distance = sqrt(pow(pixel.x,2) + pow(pixel.y,2) + pow(pixel.z,2));
        if (distance < 0.001f) {return;}

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
        if (cv::pointPolygonTest( contour, cv::Point2f((float)col, (float)row), false ) != 1) {return;}
        
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