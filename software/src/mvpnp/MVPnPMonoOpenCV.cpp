#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <iostream>
#include "MVPnPMonoOpenCV.h"

MVPnP::SolverMonoOpenCV::SolverMonoOpenCV()
{
}

MVPnP::SolverMonoOpenCV::~SolverMonoOpenCV()
{
}

bool MVPnP::SolverMonoOpenCV::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers)
{
    if( views.empty() ) throw std::runtime_error("no view was provided");

    bool ret = false;

    inliers.clear();

    if( views.front().points.size() >= 10 )
    {
        Sophus::SE3d world_to_camera = views.front().rig_to_camera * rig_to_world.inverse();

        cv::Mat rvec;
        cv::Mat tvec;
        cv::eigen2cv( world_to_camera.translation(), tvec );
        cv::eigen2cv( world_to_camera.so3().log(), rvec );

        if( use_ransac )
        {
            std::vector<uint8_t> inliers_uchar(views.front().points.size());

            ret = cv::solvePnPRansac(
                views.front().points,
                views.front().projections,
                views.front().calibration_matrix,
                views.front().distortion_coefficients,
                rvec,
                tvec,
                true,
                100,
                8.0,
                0.99,
                inliers_uchar);
                //inliers.front());

            for( int i=0; i<views.size(); i++ )
            {
                inliers[i].assign(views[i].points.size(), false);
            }

            if( inliers.front().size() != inliers_uchar.size() ) throw std::runtime_error("internal error");

            std::transform(inliers_uchar.begin(), inliers_uchar.end(), inliers.front().begin(), [] (uchar value) { return bool(value); });
        }
        else
        {
            ret = cv::solvePnP(
                views.front().points,
                views.front().projections,
                views.front().calibration_matrix,
                views.front().distortion_coefficients,
                rvec,
                tvec,
                true,
                cv::SOLVEPNP_ITERATIVE);
        }

        {
            cv::cv2eigen( tvec, world_to_camera.translation() );

            Eigen::Vector3d rodrigues;
            cv::cv2eigen( rvec, rodrigues);
            world_to_camera.so3() = Sophus::SO3d::exp(rodrigues);
        }

        rig_to_world = world_to_camera.inverse() * views.front().rig_to_camera;
    }

    return ret;
}

