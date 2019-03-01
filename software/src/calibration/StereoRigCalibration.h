
#pragma once

#include <sophus/se3.hpp>
#include <memory>
#include <string>
#include "CameraCalibration.h"

class StereoRigCalibration
{
public:

    StereoRigCalibration();

    int id;
    std::string name;
    std::string date;

    std::array<CameraCalibration,2> cameras;

    Eigen::Matrix3d computeFundamentalMatrix(int from, int to);
    Eigen::Matrix3d computeEssentialMatrix(int from, int to);

protected:

    static Eigen::Matrix3d vectorialProductMatrix(const Eigen::Vector3d& v);
};

typedef std::shared_ptr<StereoRigCalibration> StereoRigCalibrationPtr;

