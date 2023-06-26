#include "detection.h"

Detection::DetectionResults Detection::Detect(cv::Mat pcl, cv::Mat png) {

    /*
    cv::Vec3f gripper_position = Detection::detectGripper(pcl, png);
    cv::Vec3f block_position = Detection::detectBlocks(pcl, png);
    cv::Vec3f result;

    result[0] = abs(gripper_position[0] - block_position[0]);
    result[1] = abs(gripper_position[1] - block_position[1]);
    result[2] = abs(gripper_position[2] - block_position[2]);
    */

    return Detection::detectBlocks(pcl, png);
}

Detection::DetectionResults Detection::detectBlocks(cv::Mat pcl, cv::Mat png) {
    if(png.empty() or pcl.empty()){
        exit(EXIT_FAILURE);
    }
    
    //Crop Images
    cv::Mat src_image = png(setting::access.getCropRect());
    cv::Mat data_image = pcl(setting::access.getCropRect());

    //IMAGE PROCESSING PIPELINE-------------------------------------
        cv::Mat blurred;
        pipeline::removeNoise(src_image, blurred);

        //Range Operation
        cv::Mat mask;
        pipeline::generateMask(blurred, &mask);

        //Masking Operation
        cv::Mat result;
        cv::Mat pcd;
        pipeline::maskMat(blurred, result, mask);
        pipeline::maskMat(data_image, pcd, mask);

    //OBJECT RECOGNITION -------------------------------------------
        std::vector<Types::RecognitionResult> detections;
        detections = recognition::runRecognition(result);
        std::cout << "TMP: Passed detection" << std::endl;

    //CALCULATION OBJECT POSITION
        Types::RecognitionResult selected_block;
        selected_block = LocationHandler::selectDetection(detections, result.cols, result.rows);

        //recognition::drawSelected(result, selected_block);
        recognition::drawSelected(result, detections[2]);

        cv::Vec3f detection_position;
        detection_position = LocationHandler::extrapolateDetectionPosition(result, detections[2], pcd);
        
    Detection::DetectionResults extrapolate;
    extrapolate.position = detection_position;
    extrapolate.image = result;

    std::cout << "exited" << std::endl;
    return extrapolate;
}