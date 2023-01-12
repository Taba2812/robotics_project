#include "recognition.h"

void recognition::runRecognition() {
    cv::Ptr<cv::GeneralizedHoughGuil> guil = cv::createGeneralizedHoughGuil();

    recognition::setParameters(guil); 
    recognition::setDataset(guil);  

    recognition::setDataset(guil); 

    recognition::detection(guil);
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

void recognition::setDataset(cv::Ptr<cv::GeneralizedHoughGuil> guil) {
    //Generate the array of images
    std::list<cv::Mat> buffer;
    
    if (X1_Y1_Z2)
        recognition::getImagesWithRightColors(&buffer, "X1-Y1-Z2");
    
    if (X1_Y2_Z1)
        recognition::getImagesWithRightColors(&buffer, "X1-Y2-Z1");

    if (X1_Y2_Z2)
        recognition::getImagesWithRightColors(&buffer, "X1-Y2-Z2");

    if (X1_Y2_Z2_CHAMFER)
        recognition::getImagesWithRightColors(&buffer, "X1-Y2-Z2-CHAMFER");

    if (X1_Y2_Z2_TWINFILLET)
        recognition::getImagesWithRightColors(&buffer, "X1-Y2-Z2-TWINFILLET");

    if (X1_Y3_Z2)
        recognition::getImagesWithRightColors(&buffer, "X1-Y3-Z2");

    if (X1_Y3_Z2_FILLET)
        recognition::getImagesWithRightColors(&buffer, "X1-Y3-Z2-FILLET");

    if (X1_Y4_Z1)
        recognition::getImagesWithRightColors(&buffer, "X1-Y4-Z1");

    if (X1_Y4_Z2)
        recognition::getImagesWithRightColors(&buffer, "X1-Y4-Z2");

    if (X2_Y2_Z2)
        recognition::getImagesWithRightColors(&buffer, "X2-Y2-Z2");

    if (X2_Y2_Z2_FILLET)
        recognition::getImagesWithRightColors(&buffer, "X2-Y2-Z2-FILLET");
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

void recognition::detection(cv::Ptr<cv::GeneralizedHoughGuil> guil) {
    
}

void recognition::drawResults(cv::Mat input, ) {

}
