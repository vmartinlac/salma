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

        virtual ~Solver();

        virtual bool run(
            const std::vector<View>& views,
            Sophus::SE3d& rig_to_world, // initial guess and output!
            bool use_ransac,
            std::vector< std::vector<bool> >& inliers) = 0;

        static int findInliers(
            const std::vector<View>& views,
            const Sophus::SE3d& rig_to_world,
            double threshold,
            std::vector< std::vector<bool> >& inliers);

        static int extractSelection(
            const std::vector<View>& views,
            const std::vector< std::vector<bool> >& selection,
            std::vector<View>& new_views);

    protected:

        Solver();
    };

}

