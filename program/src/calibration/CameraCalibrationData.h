#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <memory>

class CameraCalibrationData
{
public:

    CameraCalibrationData();

    cv::Mat calibration_matrix;
    cv::Mat distortion_coefficients;
    cv::Size image_size;

    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);
};

typedef std::shared_ptr<CameraCalibrationData> CameraCalibrationDataPtr;

