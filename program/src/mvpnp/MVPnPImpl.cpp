#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <iostream>
#include "MVPnPImpl.h"

MVPnP::SolverImpl::SolverImpl()
{
}

MVPnP::SolverImpl::~SolverImpl()
{
}

void MVPnP::SolverImpl::applyIncrement(const TangentType& increment)
{
    Eigen::Quaterniond q = mWorldToRig.unit_quaternion();
    q.vec() += increment.segment<3>(3);
    q.w() += increment(6);
    q.normalize();

    mWorldToRig.translation() += increment.head<3>();
    mWorldToRig.setQuaternion(q);
}

double MVPnP::SolverImpl::computeError(TangentType& gradient)
{
    double error = 0.0;

    gradient.setZero();

    int count = 0;

    for( const MVPnP::View& v : *mViews )
    {
        const Sophus::SE3d world_to_camera = v.rig_to_camera * mWorldToRig;

        std::vector<cv::Point3f> points_in_camera_frame(v.points.size());

        for(int i=0; i<v.points.size(); i++)
        {
            Eigen::Vector3d in_world_frame;
            in_world_frame.x() = v.points[i].x;
            in_world_frame.y() = v.points[i].y;
            in_world_frame.z() = v.points[i].z;

            Eigen::Vector3d in_camera_frame = world_to_camera * in_world_frame;

            points_in_camera_frame[i].x = in_camera_frame.x();
            points_in_camera_frame[i].y = in_camera_frame.y();
            points_in_camera_frame[i].z = in_camera_frame.z();
        }

        std::vector<cv::Point2f> projections;
        cv::Mat jacobian;

        cv::projectPoints(
            points_in_camera_frame,
            cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F),
            v.calibration_matrix,
            v.distortion_coefficients,
            projections,
            jacobian);

        for( int i=0; i<v.points.size(); i++ )
        {
            const Eigen::Vector2d proj{ projections[i].x, projections[i].y };
            const Eigen::Vector2d proj_ref{ v.projections[i].x, v.projections[i].y };
            const Eigen::Vector2d delta_proj = proj - proj_ref;

            Eigen::Matrix<double, 2, 3> A;
            cv::cv2eigen( jacobian(cv::Range(2*i+0, 2*i+2), cv::Range(3, 6)), A);

            //gradient += 2.0 * delta_proj.transpose() * A * ;

            /*

            (t_world2rig, q_world2rig) -> X_rig -> X_camera

            X_camera = t_rig2camera + R_rig2camera * ( t_world2rig + R_world2rig * X_world )

            d_X_camera = R_rig2camera * ( d_t_world2rig + d_R_world2rig * X_world )

            */

            error += delta_proj.squaredNorm();

            count++;
        }
    }

    // divide the error and the gradient by the number of points.

    error /= double(count);
    gradient /= double(count);

    std::cout << "err = " << error << std::endl;
    //std::cout << "grad = " << gradient.transpose() << std::endl;

    return error;
}

bool MVPnP::SolverImpl::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    mViews = &views;
    mWorldToRig = rig_to_world.inverse();

    double step = 1.0e-5;

    Sophus::SE3d prev_pose = mWorldToRig;
    TangentType prev_gradient;
    double prev_err = computeError(prev_gradient);

    for(int i=0; i<10; i++)
    {
        applyIncrement(-step * prev_gradient);

        TangentType curr_gradient;
        const double curr_err = computeError(curr_gradient);

        if(curr_err < prev_err)
        {
            prev_pose = mWorldToRig;
            prev_gradient = curr_gradient;
            prev_err = curr_err;
            std::cout << i << " least square reprojection error is " << prev_err << std::endl;
            step *= 2.0;
        }
        else
        {
            mWorldToRig = prev_pose;
            step *= 0.6;
            std::cout << i << " too long a step" << std::endl;
        }
    }

    rig_to_world = mWorldToRig.inverse();

    std::cout << rig_to_world.translation() << std::endl;

    inliers.resize( views.size() );

    for( int i=0; i<views.size(); i++ )
    {
        inliers[i].assign(views[i].points.size(), true);
    }

    return false;
}

