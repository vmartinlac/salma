#pragma once

#include <stdexcept>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"

namespace Misc
{

    Eigen::Matrix3d vectorialProductMatrix( const Eigen::Vector3d& v );

    Eigen::Matrix3d computeFundamentalMatrix( CameraCalibrationDataPtr left_camera, CameraCalibrationDataPtr right_camera, StereoRigCalibrationDataPtr stereo_rig );
}

