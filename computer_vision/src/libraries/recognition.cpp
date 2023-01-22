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

    for (cv::Mat temp : *buffer) {
        guil->setTemplate(temp);
        guil->detect(grayscale, position);

        recognition::drawResults(img, position, temp.rows, temp.cols);
    }
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

void recognition::scrapOvelappingDetections(const int height, const int width, std::vector<cv::Vec4f> position) {

    //If two selections are overlapping delate the smalcheck if a bit array has just 1 oneler ones
        //How to decide if two things are overlapping?

        //If on a white canvas each detection adds 1 to the color value, you can have a texture with all the sections with more overlapping identified
        //But then how do I get back to which are overlapping, I'm not noting which are overlapping, unless
        //I use something of a bitwise operator, and I add powers of 2 based on the index of the detection then I can reach back choose which selection it was
        //So it can't be a mat it has to be an array of bits 
        // But then I can't check if there is just 1 in the space

        // How do I check if they overlap and which one do I choose to keep? Like If they overlap for less than 20%(?) keep both, otherwise keep the larger(?)
        // I can use 2 bits at the beginning of the value to determine if the cell is empty 00 | With just one 01 | Overlapping 11

    //Generating BitMap
    //I could do all that but i need to release all of this stuff, so I'm gonna go with the brute force approach because of time constrains on the project

    for (std::vector<cv::Vec4f>::iterator iter_a = position.begin(); iter_a != position.end(); ++iter_a) {
        cv::RotatedRect rect_a = cv::RotatedRect(cv::Point2f((*iter_a)[0], (*iter_a)[1]),
                                                cv::Size2f(width * (*iter_a)[2], height * (*iter_a)[2]),
                                                (*iter_a)[3]);
        int rect_a_area = width * (*iter_a)[2] * height * (*iter_a)[2];
        
        compareRotatedRects(++iter_a, position.end(), rect_a, rect_a_area, height, width);
    }
}

void compareRotatedRects(std::vector<cv::Vec4f>::iterator beginning, std::vector<cv::Vec4f>::iterator ending, cv::RotatedRect rect_to_compare, int area_to_compare, int height, int width) {
    for (std::vector<cv::Vec4f>::iterator iter = beginning; iter != ending; ++iter) {

        cv::Point a(rect_to_compare.center);
        cv::Point b((*iter)[0], (*iter)[1]);

        //Skip all calculation if center of rects are too far from each other to save on time
        if (cv::abs(cv::norm(b-a)) < height) {
            cv::RotatedRect rect_b = cv::RotatedRect(cv::Point2f((*iter)[0], (*iter)[1]),
                                                     cv::Size2f(width * (*iter)[2], height * (*iter)[2]),
                                                     (*iter)[3]);
            int rect_b_area = width * (*iter)[2] * height * (*iter)[2];

            std::array<int,8> out;
            if (cv::rotatedRectangleIntersection(rect_to_compare, rect_b, out)) {
                int intersection_area = cv::contourArea(out);
                if (area_to_compare > rect_b_area) {
                    if (intersection_area / rect_b_area > AREA_INTERSECTION_TRESHOLD) {
                        //remove rect_b
                        continue;
                    }
                } else {
                    if (intersection_area / area_to_compare > AREA_INTERSECTION_TRESHOLD) {
                        //remove rect_a
                        break;
                    }
                }
            }
        }
    }
}