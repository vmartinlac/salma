#include <QJsonDocument>
#include <QFile>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "CameraCalibration.h"

CameraCalibration::CameraCalibration()
{
}

Eigen::Matrix3d CameraCalibration::inverseOfCalibrationMatrix()
{
    const double fx = calibration_matrix.at<double>(0,0);
    const double fy = calibration_matrix.at<double>(1,1);
    const double cx = calibration_matrix.at<double>(0,2);
    const double cy = calibration_matrix.at<double>(1,2);

    Eigen::Matrix3d inv;

    inv <<
        1.0/fx, 0.0, -cx/fx,
        0.0, 1.0/fy, -cy/fy,
        0.0, 0.0, 1.0;

    return inv;
}

Eigen::Matrix3d CameraCalibration::calibrationMatrix()
{
    Eigen::Matrix3d ret;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            ret(i,j) = calibration_matrix.at<double>(i,j);
        }
    }

    return ret;
}

