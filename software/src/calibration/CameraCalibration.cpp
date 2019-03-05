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

Eigen::Matrix3d CameraCalibration::inverseOfCalibrationMatrix() const
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

Eigen::Matrix3d CameraCalibration::calibrationMatrix() const
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

QJsonValue CameraCalibration::toJson() const
{
    QJsonArray distarr;
    for(int i=0; i<distortion_coefficients.cols; i++)
    {
        distarr.push_back(distortion_coefficients.at<double>(0,i));
    }

    QJsonArray lutarr_red;
    QJsonArray lutarr_green;
    QJsonArray lutarr_blue;
    for(int i=0; i<256; i++)
    {
        const cv::Vec3f value = photometric_lut.at<cv::Vec3f>(0,i);
        lutarr_blue.push_back(value[0]);
        lutarr_green.push_back(value[1]);
        lutarr_red.push_back(value[2]);
    }

    QJsonObject lutobj;
    lutobj["red"] = lutarr_red;
    lutobj["green"] = lutarr_green;
    lutobj["blue"] = lutarr_blue;

    QJsonObject obj;
    obj["image_width"] = image_size.width;
    obj["image_height"] = image_size.height;
    obj["fx"] = calibration_matrix.at<double>(0,0);
    obj["fy"] = calibration_matrix.at<double>(1,1);
    obj["cx"] = calibration_matrix.at<double>(0,2);
    obj["cy"] = calibration_matrix.at<double>(1,2);
    obj["distortion_coefficients"] = distarr;
    obj["photometric_lut"] = lutobj;
    obj["camera_to_rig_tx"] = camera_to_rig.translation().x();
    obj["camera_to_rig_ty"] = camera_to_rig.translation().y();
    obj["camera_to_rig_tz"] = camera_to_rig.translation().z();
    obj["camera_to_rig_qx"] = camera_to_rig.unit_quaternion().x();
    obj["camera_to_rig_qy"] = camera_to_rig.unit_quaternion().y();
    obj["camera_to_rig_qz"] = camera_to_rig.unit_quaternion().z();
    obj["camera_to_rig_qw"] = camera_to_rig.unit_quaternion().w();

    return obj;
}

