
#pragma once

#include <sophus/se3.hpp>
#include <memory>
#include <string>
#include "CameraCalibrationData.h"

struct StereoRigCalibrationDataCamera
{
    Sophus::SE3d camera_to_rig;
    CameraCalibrationDataPtr calibration;
};

class StereoRigCalibrationData
{
public:

    StereoRigCalibrationData();

    int id;
    std::string name;
    std::string date;

    StereoRigCalibrationDataCamera cameras[2];

    Eigen::Matrix3d computeFundamentalMatrix(int from, int to);
    Eigen::Matrix3d computeEssentialMatrix(int from, int to);

    /*
    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);
    */

protected:

    static Eigen::Matrix3d vectorialProductMatrix(const Eigen::Vector3d& v);
};

typedef std::shared_ptr<StereoRigCalibrationData> StereoRigCalibrationDataPtr;

