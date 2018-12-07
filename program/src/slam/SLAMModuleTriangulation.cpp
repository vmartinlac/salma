#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include "TwoViewGeometry.h"
#include "SLAMModuleTriangulation.h"

SLAMModuleTriangulation::SLAMModuleTriangulation(SLAMProjectPtr project) : SLAMModule(project)
{
    mLeftCamera = project->getLeftCameraCalibration();
    mRightCamera = project->getRightCameraCalibration();
    mRig = project->getStereoRigCalibration();

    mEssentialMatrix = TwoViewGeometry::computeEssentialMatrix( mLeftCamera, mRightCamera, mRig );
    mS << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    mEssentialMatrixTilde = mS * mEssentialMatrix * mS.transpose();

    mMinAngleBetweenRays = project->getParameterReal("triangulation_min_angle_between_rays", 3.0) * M_PI / 180.0;
    mCheckPerpendicularLength = project->getParameterBoolean("triangulation_check_perpendicular_length", false);
    mPerpendicularMaxLength = project->getParameterReal("triangulation_perpendicular_max_length", 0.0);
    mInitialLifeTime = project->getParameterInteger("track_lifetime", 4);
}

void SLAMModuleTriangulation::run(FramePtr frame)
{
    for( std::pair<int,int>& p : frame->stereo_matches )
    {
        MapPointPtr world_point = triangulate( frame, p.first, p.second );

        if(world_point)
        {
            ;
        }

        if(world_point)
        {
            Projection left_proj;
            Projection right_proj;

            left_proj.mappoint = world_point;
            right_proj.mappoint = world_point;

            left_proj.point = frame->views[0].keypoints[p.first].pt;
            right_proj.point = frame->views[1].keypoints[p.second].pt;

            left_proj.max_lifetime = mInitialLifeTime;
            right_proj.max_lifetime = mInitialLifeTime;

            left_proj.type = PROJECTION_MAPPED;
            right_proj.type = PROJECTION_MAPPED;

            frame->views[0].projections.push_back( left_proj );
            frame->views[1].projections.push_back( right_proj );
        }
    }
}

MapPointPtr SLAMModuleTriangulation::triangulate(FramePtr frame, int left_keypoint, int right_keypoint)
{
    // TODO: stereo correction (Hartley ou Lindstrom).

    MapPointPtr ret;

    const std::vector<cv::Point2f> distorted_left{ frame->views[0].keypoints[left_keypoint].pt };
    const std::vector<cv::Point2f> distorted_right{ frame->views[1].keypoints[right_keypoint].pt };

    std::vector<cv::Point2f> normalized_left_arr;
    std::vector<cv::Point2f> normalized_right_arr;

    cv::undistortPoints(distorted_left, normalized_left_arr, mLeftCamera->calibration_matrix, mLeftCamera->distortion_coefficients);
    cv::undistortPoints(distorted_right, normalized_right_arr, mRightCamera->calibration_matrix, mRightCamera->distortion_coefficients);

    Eigen::Vector3d normalized_left;
    normalized_left.x() = normalized_left_arr.front().x;
    normalized_left.y() = normalized_left_arr.front().y;
    normalized_left.z() = 1.0;

    Eigen::Vector3d normalized_right;
    normalized_right.x() = normalized_right_arr.front().x;
    normalized_right.y() = normalized_right_arr.front().y;
    normalized_right.z() = 1.0;

    correctWithLindstrom( normalized_left, normalized_right );

    const Eigen::Vector3d C0 = mRig->left_camera_to_rig.translation();
    const Eigen::Matrix3d R0 = mRig->left_camera_to_rig.rotationMatrix();

    const Eigen::Vector3d C1 = mRig->right_camera_to_rig.translation();
    const Eigen::Matrix3d R1 = mRig->right_camera_to_rig.rotationMatrix();

    Eigen::Vector3d D0 = R0 * normalized_left;
    Eigen::Vector3d D1 = R1 * normalized_right;
    D0.normalize();
    D1.normalize();

    Eigen::Matrix<double, 3, 2> M;
    M.leftCols<1>() = D0;
    M.rightCols<1>() = D1;

    Eigen::Matrix2d N;
    N << -1.0, 0.0, 0.0, 1.0;

    const Eigen::Matrix2d A = M.transpose() * M * N;
    const Eigen::Vector2d Y = M.transpose() * (C0 - C1);

    const double det = A(0,0) * A(1,1) - A(0,1) * A(1,0);

    const double max_det = std::min( std::pow(std::cos(mMinAngleBetweenRays), 2.0) - 1.0, -1.0e-8 );

    if( det < max_det )
    {
        Eigen::Matrix2d invA;

        invA(0,0) = A(1,1)/det;
        invA(1,1) = A(0,0)/det;
        invA(0,1) = -A(0,1)/det;
        invA(1,0) = -A(1,0)/det;

        const Eigen::Vector2d coords = invA * Y;

        if( coords(0) > 0.0 && coords(1) > 0.0 )
        {
            const Eigen::Vector3d X0 = C0 + coords(0) * D0;
            const Eigen::Vector3d X1 = C1 + coords(1) * D1;

            if( mCheckPerpendicularLength == false || (X1-X0).norm() < mPerpendicularMaxLength )
            {
                ret.reset(new MapPoint());
                ret->position = frame->frame_to_world * ( 0.5 * (X0 + X1) );
            }
        }
    }

    return ret;
}

void SLAMModuleTriangulation::correctWithLindstrom( Eigen::Vector3d& normalized_left, Eigen::Vector3d& normalized_right )
{
    // See "Triangulation Made Easy" from Peter Lindstrom, Lawrence Livermore National Laboratory.

    Eigen::Vector3d x_left_k = normalized_left;
    Eigen::Vector3d x_right_k = normalized_right;

    const Eigen::Vector2d n_left_1 = mS * mEssentialMatrix * normalized_right;
    const Eigen::Vector2d n_right_1 = mS * mEssentialMatrix.transpose() * normalized_left;

    for(int i=0; i<3; i++)
    {
        const Eigen::Vector2d n_left_k = mS * mEssentialMatrix * x_right_k;
        const Eigen::Vector2d n_right_k = mS * mEssentialMatrix.transpose() * x_left_k;

        const double a = n_left_k.transpose() * mEssentialMatrixTilde * n_right_k;

        const double b = 0.5 * ( n_left_1.dot(n_left_k) + n_right_1.dot(n_right_k) );
        //const double b = 0.5 * ( n_left_1.transpose()*n_left_k + n_right_1.transpose()*n_right_k );

        const double c = normalized_left.transpose() * mEssentialMatrix * normalized_right;

        const double d = std::sqrt( b*b - a*c );

        const double lambda = c / ( ( b > 0.0) ? (b+d) : (b-d) );

        x_left_k = normalized_left - lambda * (mS.transpose() * n_left_k);
        x_right_k = normalized_right - lambda * (mS.transpose() * n_right_k);
    }
}

