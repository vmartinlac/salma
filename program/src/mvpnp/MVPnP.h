#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace MVPnP
{
    class View
    {
    public:

        // The three leftmost columns are 3d points in world frame.
        // The two rightmost columns are their projection.

        Eigen::Matrix<double, Eigen::Dynamic, 5, Eigen::RowMajor> points;
        cv::Mat calibration_matrix;
        cv::Mat distortion_coefficients;
        Sophus::SE3d rig_to_camera;
    };

    class Solver
    {
    public:

        bool run(
            const std::vector<View>& views,
            Sophus::SE3d& rig_to_world, // initial guess and output!
            std::vector< std::vector<bool> >& inliers);
    };
}

