#include "recognition.h"

void recognition::runRecognition(cv::InputOutputArray img) {
    cv::Ptr<cv::GeneralizedHoughGuil> guil = cv::createGeneralizedHoughGuil();
    std::list<cv::Mat> buffer;
    std::vector<cv::Vec4f> guilPosition;

    recognition::setParameters(guil); 

    recognition::setDataset(guil, &buffer); 

    recognition::detection(guil, &buffer, img, guilPosition);
}

void recognition::setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil) {
    guil->setMinDist(10);
    guil->setLevels(360);
    guil->setDp(3);
    guil->setMaxBufferSize(1000);

    guil->setMinAngle(0);
    guil->setMaxAngle(360);
    guil->setAngleStep(1);
    guil->setAngleThresh(1500);

    guil->setMinScale(0.5);
    guil->setMaxScale(2.0);
    guil->setScaleStep(0.05);
    guil->setScaleThresh(50);

    guil->setPosThresh(10);

    guil->setCannyLowThresh(30);
    guil->setCannyHighThresh(110);
}

void recognition::setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer) {
    //Generate the array of images
    
    if (X1_Y1_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y1-Z2");
    
    if (X1_Y2_Z1)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z1");

    if (X1_Y2_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2");

    if (X1_Y2_Z2_CHAMFER)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2-CHAMFER");

    if (X1_Y2_Z2_TWINFILLET)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2-TWINFILLET");

    if (X1_Y3_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y3-Z2");

    if (X1_Y3_Z2_FILLET)
        recognition::getImagesWithRightColors(buffer, "X1-Y3-Z2-FILLET");

    if (X1_Y4_Z1)
        recognition::getImagesWithRightColors(buffer, "X1-Y4-Z1");

    if (X1_Y4_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y4-Z2");

    if (X2_Y2_Z2)
        recognition::getImagesWithRightColors(buffer, "X2-Y2-Z2");

    if (X2_Y2_Z2_FILLET)
        recognition::getImagesWithRightColors(buffer, "X2-Y2-Z2-FILLET");
}

void recognition::getImagesWithRightColors(std::list<cv::Mat> *buffer, std::string block) {
    
    if (BLUE)
        recognition::getImages(buffer, block, "Blue");

    if (ORANGE)
        recognition::getImages(buffer, block, "Orange");

    if (YELLOW)
        recognition::getImages(buffer, block, "Yellow");

    if (RED)
        recognition::getImages(buffer, block, "Red");

    if (VIOLET)
        recognition::getImages(buffer, block, "Violet");
}

void recognition::getImages(std::list<cv::Mat> *buffer, std::string block, std::string color) {
    int latitude[] = {-90, -60, -45, -30, 0, 30, 45, 60, 90};
    int longitude[] = {0, 30, 45, 60};

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 4; j++) {
            std::string filename = "/" + block + "/" + "Blocco_" + block + "_" + color + "_" + std::to_string(latitude[i]) + "-" + std::to_string(longitude[j]) + ".png";
            buffer->push_front(cv::imread(DATASET_PATH + filename, cv::IMREAD_GRAYSCALE));
        }
    }
}

void recognition::detection(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer, cv::InputOutputArray img, std::vector<cv::Vec4f> position) {
    cv::Mat grayscale;
    cv::cvtColor(img, grayscale, cv::COLOR_RGB2GRAY);
    int total_detections = 0;

    for (cv::Mat temp : *buffer) {
        std::vector<cv::Vec4f> partial_detections;
        guil->setTemplate(temp);
        guil->detect(grayscale, partial_detections);

        position.insert(position.end(), partial_detections.begin(), partial_detections.end());
        partial_detections.clear();
    }

    /*
    REFACTOR OVERLAP CHECK AND DRAWRESULTS

    recognition::scrapOvelappingDetections(&partial_detections, temp.cols, temp.rows);
    recognition::drawResults(img, partial_detections, temp.rows, temp.cols);
    */

    std::cout << "Total detections across all images: " << std::to_string(position.size()) << std::endl;
}

void recognition::drawResults(cv::InputOutputArray img, std::vector<cv::Vec4f> position, int template_h, int template_w) {
    
    for (std::vector<cv::Vec4f>::iterator iter = position.begin(); iter != position.end(); ++iter) {
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f((*iter)[0], (*iter)[1]),
                                                cv::Size2f(template_w * (*iter)[2], template_h * (*iter)[2]),
                                                (*iter)[3]);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        
        cv::circle(img, cv::Point((*iter)[0], (*iter)[1]), 1, cv::Scalar(255,255,255), 1);   
    }
}

void recognition::scrapOvelappingDetections(std::vector<cv::Vec4f> *detections, int width, int height) {

    if ((*detections).empty()) {return;}

    std::vector<std::vector<cv::Vec4f>::iterator> to_delete;
    int size = (*detections).size();

    std::cout << "Size: " << std::to_string(size) << std::endl;

    for (std::vector<cv::Vec4f>::iterator element = (*detections).begin(); element != std::prev((*detections).end()); ++element) {
        std::cout << "External Iteration" << std::endl;

        cv::Vec4f obj_a = *element;
        cv::RotatedRect rect_a = cv::RotatedRect(cv::Point2f(obj_a[0], obj_a[1]), cv::Size2f(width * obj_a[2], height * obj_a[2]), obj_a[3]);
        int rect_a_area = width * height * obj_a[2] * obj_a[2];

        for (std::vector<cv::Vec4f>::iterator comparison = element + 1; comparison != (*detections).end(); ++comparison) {
            std::cout << "  Internal Iteration" << std::endl;

            cv::Vec4f obj_b = *comparison;
            if (recognition::distanceCondition(obj_a, obj_b, width, height)) {continue;}

            cv::RotatedRect rect_b = cv::RotatedRect(cv::Point2f(obj_b[0], obj_b[1]), cv::Size2f(width * obj_b[2], height * obj_b[2]), obj_b[3]);

            int rect_b_area = width * height * obj_b[2] * obj_b[2];

            std::vector<cv::Point2f> out;

            if (cv::rotatedRectangleIntersection(rect_a, rect_b, out)) {
                int intersection_area = cv::contourArea(out);

                if (rect_a_area > rect_b_area) {
                    std::cout << "      Intersection: " << std::to_string(intersection_area / rect_b_area) << std::endl;
                    if (intersection_area / rect_b_area > AREA_INTERSECTION_TRESHOLD)
                        to_delete.push_back(comparison);
                } else {
                    std::cout << "      Intersection: " << std::to_string(intersection_area / rect_a_area) << std::endl;
                    if (intersection_area / rect_a_area > AREA_INTERSECTION_TRESHOLD)
                        to_delete.push_back(element);
                }
            }
        }
    }

    if (to_delete.empty()) {return;}
    std::cout << "----Found " << std::to_string(to_delete.size()) << " detections to delete" << std::endl;

    for (std::vector<std::vector<cv::Vec4f>::iterator>::iterator iter = to_delete.begin(); iter != to_delete.end(); ++iter) {
        (*detections).erase(*iter);
    }
}

bool recognition::distanceCondition(cv::Vec4f first, cv::Vec4f second, int width, int height) {
    
    int distance = sqrt(pow(second[0]-first[0],2) + pow(second[1]-first[1],2));

    int max_dist = sqrt(pow(width, 2) + pow(height, 2));

    if (first[3] > second[3]) {
        max_dist *= (float)first[2] / 2;
    } else {
        max_dist *= (float)second[2] / 2;
    }

    return distance > max_dist;
}