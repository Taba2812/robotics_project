#ifndef TEMP_HANDLER
#define TEMP_HANDLER

#include <fstream>
#include <opencv2/core.hpp>

namespace TempFileHandler {

    bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
    bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
    bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
    bool LoadMatBinary(const std::string& filename, cv::Mat& output);

}

#endif
