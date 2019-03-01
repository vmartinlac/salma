#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <memory>

class CameraCalibration
{
public:

    CameraCalibration();

    cv::Mat calibration_matrix;
    cv::Mat distortion_coefficients;
    cv::Size image_size;
    Sophus::SE3d camera_to_rig;
    std::array<float,256> photometric_lut;

public:

    Eigen::Matrix3d inverseOfCalibrationMatrix();
    Eigen::Matrix3d calibrationMatrix();
};

typedef std::shared_ptr<CameraCalibration> CameraCalibrationPtr;

