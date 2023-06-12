#include "detection.h"

cv::Vec3f Detection::Detect(cv::Mat pcl, cv::Mat png) {
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

cv::Vec3f Detection::detectGripper(cv::Mat pcl, cv::Mat png) {
    //Crop Images
    cv::Mat src_image  (png, setting::access.getCropRect(png));
    cv::Mat data_image (pcl, setting::access.getCropRect(pcl));

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

    return detection_position;
}

cv::Vec3f Detection::detectBlocks(cv::Mat pcl, cv::Mat png) {
    //Crop Images
    cv::Mat src_image  (png, setting::access.getCropRect(png));
    cv::Mat data_image (pcl, setting::access.getCropRect(pcl));

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
        detections = recognition::runRecognition(result);

    //CALCULATION OBJECT POSITION
        cv::Vec4f selected_block;
        selected_block = LocationHandler::selectDetection(detections, result.cols, result.rows);

        cv::Vec3f detection_position;
        detection_position = LocationHandler::extrapolateDetectionPosition(selected_block, pcd, pcd.size());

    return detection_position;
}