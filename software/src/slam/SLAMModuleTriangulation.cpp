#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include "SLAMModuleTriangulation.h"

SLAMModuleTriangulation::SLAMModuleTriangulation(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleTriangulation::~SLAMModuleTriangulation()
{
}

bool SLAMModuleTriangulation::init()
{
    SLAMContextPtr con = context();

    mLeftCamera = con->calibration->cameras[0].calibration;
    mRightCamera = con->calibration->cameras[1].calibration;
    mRig = con->calibration;

    mEssentialMatrix = mRig->computeEssentialMatrix(0, 1);
    mS << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    mEssentialMatrixTilde = mS * mEssentialMatrix * mS.transpose();

    mMinAngleBetweenRays = con->configuration->triangulation_min_angle_between_rays;
    mCheckPerpendicularLength = con->configuration->triangulation_check_perpendicular_length;
    mPerpendicularMaxLength = con->configuration->triangulation_perpendicular_max_length;
    mMaxReprojectionError = con->configuration->triangulation_max_reprojection_error;
    mInitialLifeTime = con->configuration->triangulation_track_lifetime;
    mUseLindstrom = con->configuration->triangulation_use_lindstrom;

    return true;
}

void SLAMModuleTriangulation::operator()()
{
    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    for( std::pair<int,int>& p : frame->stereo_matches )
    {
        SLAMMapPointPtr world_point = triangulate( frame, p.first, p.second );

        if(world_point)
        {
            SLAMProjection left_proj;
            SLAMProjection right_proj;

            left_proj.mappoint = world_point;
            right_proj.mappoint = world_point;

            left_proj.point = frame->views[0].keypoints[p.first].pt;
            right_proj.point = frame->views[1].keypoints[p.second].pt;

            left_proj.max_lifetime = mInitialLifeTime;
            right_proj.max_lifetime = mInitialLifeTime;

            left_proj.type = SLAM_PROJECTION_MAPPED;
            right_proj.type = SLAM_PROJECTION_MAPPED;

            frame->views[0].projections.push_back( left_proj );
            frame->views[1].projections.push_back( right_proj );
        }
    }
}

SLAMMapPointPtr SLAMModuleTriangulation::triangulate(SLAMFramePtr frame, int left_keypoint, int right_keypoint)
{
    // Declare some variables.

    bool ok = true;
    Eigen::Vector3d result;
    SLAMMapPointPtr ret;

    // Compute normalized points.

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

    // Computed corrected normalized points with Lindstrom method.

    if( mUseLindstrom )
    {
        //std::cout << normalized_left.transpose() << " " << normalized_right.transpose() << std::endl;
        correctWithLindstrom( normalized_left, normalized_right );
        //std::cout << normalized_left.transpose() << " " << normalized_right.transpose() << std::endl;
    }

    // Triangulate first in rig frame. It will be transformed to world frame at the very end of this function.
    // We use the middle of the common perpendicular of the two rays.

    const Eigen::Vector3d C0 = mRig->cameras[0].camera_to_rig.translation();
    const Eigen::Matrix3d R0 = mRig->cameras[0].camera_to_rig.rotationMatrix();

    const Eigen::Vector3d C1 = mRig->cameras[1].camera_to_rig.translation();
    const Eigen::Matrix3d R1 = mRig->cameras[1].camera_to_rig.rotationMatrix();

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

    ok = ( det < max_det );

    if(ok)
    {
        ok = false;

        Eigen::Matrix2d invA;

        invA(0,0) = A(1,1)/det;
        invA(1,1) = A(0,0)/det;
        invA(0,1) = -A(0,1)/det;
        invA(1,0) = -A(1,0)/det;

        const Eigen::Vector2d coords = invA * Y;
        //std::cout << coords.transpose() << std::endl;

        if( coords(0) > 0.0 && coords(1) > 0.0 )
        {
            const Eigen::Vector3d X0 = C0 + coords(0) * D0;
            const Eigen::Vector3d X1 = C1 + coords(1) * D1;

            if( mCheckPerpendicularLength == false || (X1-X0).norm() < mPerpendicularMaxLength )
            {
                result = 0.5 * (X0 + X1);
                ok = true;
            }
        }
    }

    // Check reprojection.

    if(ok)
    {
        const Eigen::Vector3d in_left_camera = mRig->cameras[0].camera_to_rig.inverse() * result;
        const Eigen::Vector3d in_right_camera = mRig->cameras[1].camera_to_rig.inverse() * result;

        ok = (in_left_camera.z() > 0.0 && in_right_camera.z() > 0.0);

        if(ok)
        {
            cv::Point3f in_left_camera_cv;
            in_left_camera_cv.x = in_left_camera.x();
            in_left_camera_cv.y = in_left_camera.y();
            in_left_camera_cv.z = in_left_camera.z();

            cv::Point3f in_right_camera_cv;
            in_right_camera_cv.x = in_right_camera.x();
            in_right_camera_cv.y = in_right_camera.y();
            in_right_camera_cv.z = in_right_camera.z();

            std::vector<cv::Point3f> in_left_camera_arr{ in_left_camera_cv };
            std::vector<cv::Point3f> in_right_camera_arr{ in_right_camera_cv };

            std::vector<cv::Point2f> projected_left;
            std::vector<cv::Point2f> projected_right;

            cv::projectPoints(
                in_left_camera_arr,
                cv::Mat::zeros(3,1,CV_64F),
                cv::Mat::zeros(3,1,CV_64F),
                mLeftCamera->calibration_matrix,
                mLeftCamera->distortion_coefficients,
                projected_left);

            cv::projectPoints(
                in_right_camera_arr,
                cv::Mat::zeros(3,1,CV_64F),
                cv::Mat::zeros(3,1,CV_64F),
                mRightCamera->calibration_matrix,
                mRightCamera->distortion_coefficients,
                projected_right);

            const double error_left = cv::norm( projected_left.front() - distorted_left.front() );
            const double error_right = cv::norm( projected_right.front() - distorted_right.front() );

            //std::cout << error_left << " " << error_right << std::endl;

            ok = (error_left < mMaxReprojectionError && error_right < mMaxReprojectionError);
        }
    }

    // Convert point from rig frame to world frame.

    if(ok)
    {
        ret.reset(new SLAMMapPoint());
        ret->position = frame->frame_to_world * result;
    }

    return ret;
}

void SLAMModuleTriangulation::correctWithLindstrom( Eigen::Vector3d& normalized_left, Eigen::Vector3d& normalized_right )
{
/*
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
*/
}

