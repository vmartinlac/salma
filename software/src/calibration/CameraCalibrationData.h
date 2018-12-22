#pragma once

#include <Eigen/Eigen>
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
    std::string name;

    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);

    Eigen::Matrix3d inverseOfCalibrationMatrix();
    Eigen::Matrix3d calibrationMatrix();
};

typedef std::shared_ptr<CameraCalibrationData> CameraCalibrationDataPtr;

