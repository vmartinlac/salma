
#pragma once

#include <sophus/se3.hpp>
#include <string>

class StereoRigCalibrationData
{
public:

    StereoRigCalibrationData();

    Sophus::SE3<double> left_camera_to_world;
    Sophus::SE3<double> right_camera_to_world;

    bool saveToFile(const std::string& path);
    bool loadFromFile(const std::string& path);
};

