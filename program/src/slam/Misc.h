#pragma once

#include <stdexcept>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"

namespace Misc
{
    template<int rows, int cols>
    Eigen::Matrix<double,rows,cols> matToEigen( const cv::Mat& m );

    Eigen::Matrix3d vectorialProductMatrix( const Eigen::Vector3d& v );

    Eigen::Matrix3d computeFundamentalMatrix( CameraCalibrationDataPtr left_camera, CameraCalibrationDataPtr right_camera, StereoRigCalibrationDataPtr stereo_rig );
}

template<int rows, int cols>
Eigen::Matrix<double,rows,cols> Misc::matToEigen(const cv::Mat& m)
{
    Eigen::Matrix<double,rows,cols> ret;

    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            switch( m.type() )
            {
            case CV_64F:
                ret(i,j) = m.at<double>(i,j);
                break;
            case CV_32F:
                ret(i,j) = m.at<float>(i,j);
                break;
            default:
                throw std::runtime_error("internal error");
            }
        }
    }

    return ret;
}

