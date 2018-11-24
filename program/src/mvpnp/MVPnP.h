#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace MVPnP
{

    class View
    {
    public:

        std::vector<cv::Point3f> points;
        std::vector<cv::Point2f> projections;
        cv::Mat calibration_matrix;
        cv::Mat distortion_coefficients;
        Sophus::SE3d rig_to_camera;
    };

    class Solver
    {
    public:

        static Solver* create();

        virtual ~Solver();

        virtual bool run(
            const std::vector<View>& views,
            Sophus::SE3d& rig_to_world, // initial guess and output!
            std::vector< std::vector<bool> >& inliers) = 0;

    protected:

        Solver();
    };

}

