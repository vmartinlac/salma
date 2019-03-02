#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <QJsonValue>
#include <sophus/se3.hpp>
#include <string>
#include <memory>

class CameraCalibration
{
public:

    cv::Mat calibration_matrix;
    cv::Mat distortion_coefficients;
    cv::Size image_size;
    Sophus::SE3d camera_to_rig;
    cv::Mat photometric_lut;

public:

    CameraCalibration();

    Eigen::Matrix3d inverseOfCalibrationMatrix() const;
    Eigen::Matrix3d calibrationMatrix() const;

    QJsonValue toJson() const;
};

typedef std::shared_ptr<CameraCalibration> CameraCalibrationPtr;

