
#pragma once

#include <sophus/se3.hpp>
#include <memory>
#include <string>

class StereoRigCalibrationData
{
public:

    StereoRigCalibrationData();

    Sophus::SE3d left_camera_to_rig;
    Sophus::SE3d right_camera_to_rig;

    std::string name;

    //Eigen::Matrix3d fundamental_matrix;
    //Eigen::Matrix3d essential_matrix;

    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);
};

typedef std::shared_ptr<StereoRigCalibrationData> StereoRigCalibrationDataPtr;

