#include "recognition.h"

std::vector<cv::Vec4f> recognition::runRecognition(cv::InputOutputArray img) {
    cv::Ptr<cv::GeneralizedHoughGuil> guil = cv::createGeneralizedHoughGuil();
    std::list<cv::Mat> buffer;
    std::vector<cv::Vec4f> guilPosition;

    recognition::setParameters(guil); 

    recognition::setDataset(guil, &buffer);
    std::cout << "Number of images used as templates: " << buffer.size() << std::endl;

    return recognition::detection(guil, &buffer, img, guilPosition);
}

void recognition::setParameters(cv::Ptr<cv::GeneralizedHoughGuil> guil) {
    guil->setMinDist(setting::access.guil.min_dist);
    guil->setLevels(setting::access.guil.levels);
    guil->setDp(setting::access.guil.dp);
    guil->setMaxBufferSize(setting::access.guil.max_buffer_size);

    guil->setMinAngle(setting::access.guil.min_angle);
    guil->setMaxAngle(setting::access.guil.max_angle);
    guil->setAngleStep(setting::access.guil.step_angle);
    guil->setAngleThresh(setting::access.guil.thresh_angle);

    guil->setMinScale(setting::access.guil.min_scale);
    guil->setMaxScale(setting::access.guil.max_scale);
    guil->setScaleStep(setting::access.guil.step_scale);
    guil->setScaleThresh(setting::access.guil.thresh_scale);

    guil->setPosThresh(setting::access.guil.thersh_pos);

    guil->setCannyLowThresh(setting::access.guil.canny_thresh_low);
    guil->setCannyHighThresh(setting::access.guil.canny_thresh_high);
}

void recognition::setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer) {
    //Generate the array of images
    
    if (setting::access.X1_Y1_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y1-Z2");
    
    if (setting::access.X1_Y2_Z1)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z1");

    if (setting::access.X1_Y2_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2");

    if (setting::access.X1_Y2_Z2_CHAMFER)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2-CHAMFER");

    if (setting::access.X1_Y2_Z2_TWINFILLET)
        recognition::getImagesWithRightColors(buffer, "X1-Y2-Z2-TWINFILLET");

    if (setting::access.X1_Y3_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y3-Z2");

    if (setting::access.X1_Y3_Z2_FILLET)
        recognition::getImagesWithRightColors(buffer, "X1-Y3-Z2-FILLET");

    if (setting::access.X1_Y4_Z1)
        recognition::getImagesWithRightColors(buffer, "X1-Y4-Z1");

    if (setting::access.X1_Y4_Z2)
        recognition::getImagesWithRightColors(buffer, "X1-Y4-Z2");

    if (setting::access.X2_Y2_Z2)
        recognition::getImagesWithRightColors(buffer, "X2-Y2-Z2");

    if (setting::access.X2_Y2_Z2_FILLET)
        recognition::getImagesWithRightColors(buffer, "X2-Y2-Z2-FILLET");
}

void recognition::getImagesWithRightColors(std::list<cv::Mat> *buffer, std::string block) {
    
    if (setting::access.BLUE)
        recognition::getImages(buffer, block, "Blue");

    if (setting::access.ORANGE)
        recognition::getImages(buffer, block, "Orange");

    if (setting::access.YELLOW)
        recognition::getImages(buffer, block, "Yellow");

    if (setting::access.RED)
        recognition::getImages(buffer, block, "Red");

    if (setting::access.VIOLET)
        recognition::getImages(buffer, block, "Violet");
}

void recognition::getImages(std::list<cv::Mat> *buffer, std::string block, std::string color) {
    int latitude[] = {-90, -60, -45, -30, 0, 30, 45, 60, 90};
    int longitude[] = {0, 30, 45, 60};

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 4; j++) {
            std::string filename = "/" + block + "/" + "Blocco_" + block + "_" + color + "_" + std::to_string(latitude[i]) + "-" + std::to_string(longitude[j]) + ".png";
            buffer->push_front(cv::imread(setting::access.DATASET_PATH + filename, cv::IMREAD_GRAYSCALE));
        }
    }
}

std::vector<cv::Vec4f> recognition::detection(cv::Ptr<cv::GeneralizedHoughGuil> guil, std::list<cv::Mat> *buffer, cv::InputOutputArray img, std::vector<cv::Vec4f> position) {
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

    std::cout << "pos_0: " << position[0][0] << " pos_1: " << position[0][1] << std::endl;

    return position;

}

void recognition::drawResults(cv::InputOutputArray img, std::vector<cv::Vec4f> position, std::list<cv::Mat> *buffer, std::vector<cv::Mat> pTemplate) {
    if (position.size() != pTemplate.size()) {return;}
    
    //ITERATION BOUNDARIES
    std::vector<cv::Vec4f>::iterator pos_iter = position.begin();
    std::vector<cv::Mat>::iterator temp_iter = pTemplate.begin();
    
    int i = 0;
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

void recognition::drawSelected(cv::Mat img, cv::Vec4f selection) {
    int size = 10;
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(selection[0], selection[1]),
                                            cv::Size2f(size * selection[2], size * selection[2]),
                                            selection[3]);

    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
            cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
}

void recognition::scrapOvelappingDetections(std::vector<cv::Vec4f> *detections, std::vector<cv::Mat> *pTemplate) {

    if (detections->empty() || pTemplate->empty()) {return;}
    if (detections->size()  != pTemplate->size())  {return;}

    std::cout << "[Detection][ScrapOverlyingDetections] Function Barrier Checks Passed!" << std::endl;
    std::cout << "[Detection][ScrapOverlyingDetections] inter_tresh=" << setting::access.intersection_threshold << std::endl;

    std::vector<cv::Vec4f>::iterator outer_pos_iter = detections->begin();
    std::vector<cv::Mat>::iterator outer_temp_iter = pTemplate->begin();
    bool need_to_increment = true;
    
    int inner_counter = 0;
    int outer_counter = 0;
    while (outer_pos_iter != std::prev(detections->end()) and outer_pos_iter != detections->end()) {
        
        //std::cout << "[Detection][ScrapOverlyingDetections][Outer Loop #" << outer_counter << "]" <<std::endl;
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

        inner_counter = outer_counter + 1;

        while (inner_pos_iter != detections->end() and inner_temp_iter != pTemplate->end()) {

            //std::cout << "[Detection][ScrapOverlyingDetections] [Inner Loop #" << inner_counter << "]" <<std::endl;
            if (outer_pos_iter == detections->end() and outer_temp_iter == pTemplate->end()) {
                need_to_increment = false;
                break;
            }

            cv::Vec4f comparison = *inner_pos_iter;
            cv::Mat comparison_temp = *inner_temp_iter;

            if (recognition::distanceCondition(element, element_temp.size(), comparison, comparison_temp.size())) 
                {++inner_pos_iter;++inner_temp_iter;inner_counter++;continue;}

            cv::RotatedRect comparison_rect = cv::RotatedRect (cv::Point2f(comparison[0], comparison[1]), 
                                                                cv::Size2f(comparison_temp.cols * comparison[2], comparison_temp.rows * comparison[2]), 
                                                                comparison[3]);
            int comparison_rect_area = comparison_temp.cols * comparison_temp.rows * comparison[2] * comparison[2];

            std::vector<cv::Point2f> out;
            if (cv::rotatedRectangleIntersection(element_rect, comparison_rect, out)) {
                int intersection_area = cv::contourArea(out);
                if (element_rect_area > comparison_rect_area) {
                    if ((double)intersection_area / (double)comparison_rect_area > setting::access.intersection_threshold) {
                        inner_pos_iter = detections->erase(inner_pos_iter);
                        inner_temp_iter = pTemplate->erase(inner_temp_iter);
                        inner_counter++;
                        continue;
                    }
                } else {
                    if ((double)intersection_area / (double)element_rect_area > setting::access.intersection_threshold) {
                        outer_pos_iter = detections->erase(outer_pos_iter);
                        outer_temp_iter = pTemplate->erase(outer_temp_iter);
                        outer_counter++;
                        need_to_increment = false;
                        break;
                    }
                }
            }
            //INCREMENTS 
            ++inner_pos_iter;
            ++inner_temp_iter; 
            ++inner_counter;       
        }

        if (need_to_increment) {
            ++outer_pos_iter;
            ++outer_temp_iter;
            ++outer_counter;
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