#pragma once

#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"

namespace TwoViewGeometry
{
    Eigen::Matrix3d computeFundamentalMatrix(
        CameraCalibrationDataPtr left_camera,
        CameraCalibrationDataPtr right_camera,
        StereoRigCalibrationDataPtr stereo_rig );

    Eigen::Matrix3d computeEssentialMatrix(
        CameraCalibrationDataPtr left_camera,
        CameraCalibrationDataPtr right_camera,
        StereoRigCalibrationDataPtr stereo_rig );

    Eigen::Matrix3d vectorialProductMatrix( const Eigen::Vector3d& v );
};

