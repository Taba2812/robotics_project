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

Detection::DetectionResults Detection::detectGripper(cv::Mat pcl, cv::Mat png) {
    //Crop Images
    cv::Mat src_image  (png, setting::access.getCropRect());
    cv::Mat data_image (pcl, setting::access.getCropRect());

    if(src_image.empty() or data_image.empty()){
        exit(EXIT_FAILURE);
    }

    cv::Size src_size = cv::Size((int)src_image.cols, (int)src_image.rows);

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
        std::vector<cv::Vec4f> detections;
        //!NOTE CHANGE WITH GRIPPER RECOGNITION WITH DIFFERENT TEMPLATES
        detections = recognition::runRecognition(result);

    //CALCULATION OBJECT POSITION
        cv::Vec4f selected_block;
        selected_block = LocationHandler::selectDetection(detections, result.cols, result.rows);

        cv::Vec3f detection_position;
        detection_position = LocationHandler::extrapolateDetectionPosition(selected_block, pcd, pcd.size());

    Detection::DetectionResults extrapolate;
    extrapolate.position = detection_position;
    extrapolate.image = result;

    return extrapolate;
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
        std::vector<cv::Vec4f> detections;
        detections = recognition::runRecognition(result);
        std::cout << "TMP: Passed detection" << std::endl;

    //CALCULATION OBJECT POSITION
        cv::Vec4f selected_block;
        selected_block = LocationHandler::selectDetection(detections, result.cols, result.rows);

        //recognition::drawSelected(result, selected_block);
        recognition::drawSelected(result, detections[2]);

        cv::Vec3f detection_position;
        //detection_position = LocationHandler::extrapolateDetectionPosition(selected_block, pcd, pcd.size());
        detection_position = LocationHandler::extrapolateDetectionPosition(result, detections[2], pcd, pcd.size());
        
    Detection::DetectionResults extrapolate;
    extrapolate.position = detection_position;
    extrapolate.image = result;

    std::cout << "exited" << std::endl;
    return extrapolate;
}