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
    std::vector<cv::Mat> pTemplate;

    for (cv::Mat temp : *buffer) {
        std::vector<cv::Vec4f> partial_detections;
        guil->setTemplate(temp);
        guil->detect(grayscale, partial_detections);

        position.insert(position.end(), partial_detections.begin(), partial_detections.end());
        for (int i = 0; i < partial_detections.size(); i++) {
            pTemplate.push_back(temp);
        }

        partial_detections.clear();
    }

    std::cout << "Total detections across all images: " << std::to_string(position.size()) << std::endl;

    recognition::scrapOvelappingDetections(&position, &pTemplate);

    std::cout << "Total detections after scrapping: " << std::to_string(position.size()) << std::endl;

    recognition::drawResults(img, position, buffer, pTemplate);
}

void recognition::drawResults(cv::InputOutputArray img, std::vector<cv::Vec4f> position, std::list<cv::Mat> *buffer, std::vector<cv::Mat> pTemplate) {
    if (position.size() != pTemplate.size()) {return;}
    
    //ITERATION BOUNDARIES
    std::vector<cv::Vec4f>::iterator pos_iter = position.begin();
    std::vector<cv::Mat>::iterator temp_iter = pTemplate.begin();
    
    for (; pos_iter != position.end();) {
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f((*pos_iter)[0], (*pos_iter)[1]),
                                                cv::Size2f((*temp_iter).cols * (*pos_iter)[2], (*temp_iter).rows * (*pos_iter)[2]),
                                                (*pos_iter)[3]);

        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        
        cv::circle(img, cv::Point((*pos_iter)[0], (*pos_iter)[1]), 1, cv::Scalar(255,255,255), 1); 

        //INCREMENTS
        ++pos_iter;
        ++temp_iter;
    }
}

void recognition::scrapOvelappingDetections(std::vector<cv::Vec4f> *detections, std::vector<cv::Mat> *pTemplate) {

    if (detections->empty() || pTemplate->empty()) {return;}
    if (detections->size()  != pTemplate->size())  {return;}

    std::vector<cv::Vec4f>::iterator outer_pos_iter = detections->begin();
    std::vector<cv::Mat>::iterator outer_temp_iter = pTemplate->begin();
    bool need_to_increment = true;
    
    while (outer_pos_iter != std::prev(detections->end())) {
        
        need_to_increment = true;

        cv::Vec4f element = *outer_pos_iter;
        cv::Mat element_temp = *outer_temp_iter;

        cv::RotatedRect element_rect = cv::RotatedRect (cv::Point2f(element[0], element[1]), 
                                                        cv::Size2f(element_temp.cols * element[2], element_temp.rows * element[2]), 
                                                        element[3]);
        int element_rect_area = element_temp.cols * element_temp.rows * element[2] * element[2];

        //SECOND LOOP
        std::vector<cv::Vec4f>::iterator inner_pos_iter = outer_pos_iter + 1;
        std::vector<cv::Mat>::iterator inner_temp_iter = outer_temp_iter + 1;

        while (inner_pos_iter != detections->end()) {

            cv::Vec4f comparison = *inner_pos_iter;
            cv::Mat comparison_temp = *inner_temp_iter;

            if (recognition::distanceCondition(element, element_temp.size(), comparison, comparison_temp.size())) 
                {++inner_pos_iter;++inner_temp_iter;continue;}

            cv::RotatedRect comparison_rect = cv::RotatedRect (cv::Point2f(comparison[0], comparison[1]), 
                                                                cv::Size2f(comparison_temp.cols * comparison[2], comparison_temp.rows * comparison[2]), 
                                                                comparison[3]);
            int comparison_rect_area = comparison_temp.cols * comparison_temp.rows * comparison[2] * comparison[2];
            
            std::vector<cv::Point2f> out;
            if (cv::rotatedRectangleIntersection(element_rect, comparison_rect, out)) {
                int intersection_area = cv::contourArea(out);
                if (element_rect_area > comparison_rect_area) {
                    if ((double)intersection_area / (double)comparison_rect_area > AREA_INTERSECTION_TRESHOLD) {
                        inner_pos_iter = detections->erase(inner_pos_iter);
                        inner_temp_iter = pTemplate->erase(inner_temp_iter);
                        continue;
                    }
                } else {
                    if ((double)intersection_area / (double)element_rect_area > AREA_INTERSECTION_TRESHOLD) {
                        outer_pos_iter = detections->erase(outer_pos_iter);
                        outer_temp_iter = pTemplate->erase(outer_temp_iter);
                        need_to_increment = false;
                        break;
                    }
                }
            }
            //INCREMENTS 
            ++inner_pos_iter;
            ++inner_temp_iter;          
        }

        if (need_to_increment) {
            ++outer_pos_iter;
            ++outer_temp_iter;
        }       
    }
}

bool recognition::distanceCondition(cv::Vec4f first, cv::Size2i first_size, cv::Vec4f second, cv::Size2i second_size) {
    
    int distance = sqrt(pow(second[0]-first[0],2) + pow(second[1]-first[1],2));

    int max_dist = 1;
    if (first_size.area() > second_size.area()) {
        max_dist = (sqrt(pow(first_size.width, 2) + pow(first_size.height, 2)) * (double)first[2]) / 2;
    } else {
        max_dist = (sqrt(pow(second_size.width, 2) + pow(second_size.height, 2)) * (double)second[2]) / 2;
    }

    return distance > max_dist;
}