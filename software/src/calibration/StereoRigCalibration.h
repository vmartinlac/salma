
#pragma once

#include <QJsonValue>
#include <sophus/se3.hpp>
#include <memory>
#include <string>
#include "CameraCalibration.h"

class StereoRigCalibration
{
public:

    int id;
    std::string name;
    std::string date;

    std::array<CameraCalibration,2> cameras;

public:

    StereoRigCalibration();

    Eigen::Matrix3d computeFundamentalMatrix(int from, int to) const;
    Eigen::Matrix3d computeEssentialMatrix(int from, int to) const;

    QJsonValue toJson() const;

protected:

    static Eigen::Matrix3d vectorialProductMatrix(const Eigen::Vector3d& v);
};

typedef std::shared_ptr<StereoRigCalibration> StereoRigCalibrationPtr;

