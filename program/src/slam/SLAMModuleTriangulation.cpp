#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include "SLAMModuleTriangulation.h"

SLAMModuleTriangulation::SLAMModuleTriangulation(SLAMProjectPtr project) : SLAMModule(project)
{
    mLeftCamera = project->getLeftCameraCalibration();
    mRightCamera = project->getRightCameraCalibration();
    mRig = project->getStereoRigCalibration();

    mMinAngleBetweenRays = project->getParameterReal("triangulation_min_angle_between_rays", 3.0) * M_PI / 180.0;
    mCheckPerpendicularLength = project->getParameterBoolean("triangulation_check_perpendicular_length", false);
    mPerpendicularMaxLength = project->getParameterReal("triangulation_perpendicular_max_length", 0.0);
}

void SLAMModuleTriangulation::run(FramePtr frame)
{
    if( frame->previous_frame )
    {
        std::vector<Track>& left_tracks = frame->views[0].tracks;
        std::vector<Track>& right_tracks = frame->views[1].tracks;

        std::vector<Track>& prev_left_tracks = frame->previous_frame->views[0].tracks;
        std::vector<Track>& prev_right_tracks = frame->previous_frame->views[1].tracks;

        for(Track& t : left_tracks)
        {
            if( t.anterior_match >= 0 )
            {
                t.mappoint = prev_left_tracks[t.anterior_match].mappoint;
            }
        }

        for(Track& t : right_tracks)
        {
            if( t.anterior_match >= 0 )
            {
                t.mappoint = prev_right_tracks[t.anterior_match].mappoint;
            }
        }

        for(Track& t : left_tracks)
        {
            if( t.stereo_match >= 0 )
            {
                MapPointPtr& left_point = t.mappoint;
                MapPointPtr& right_point = right_tracks[t.stereo_match].mappoint;

                if( bool(left_point) && bool(right_point) )
                {
                }
                else if( bool(left_point) )
                {
                    right_point = left_point;
                }
                else if( bool(right_point) )
                {
                    left_point = right_point;
                }
                else if( t.anterior_match >= 0 && right_tracks[t.stereo_match].anterior_match >= 0 )
                {
                    right_point = left_point = triangulate(
                        frame->previous_frame,
                        t.anterior_match,
                        right_tracks[t.stereo_match].anterior_match);
                }
            }
        }
    }
}

MapPointPtr SLAMModuleTriangulation::triangulate(FramePtr frame, int left_keypoint, int right_keypoint)
{
    // TODO: stereo correction (Hartley ou Lindstrom).

    MapPointPtr ret;

    const std::vector<cv::Point2f> distorted_left{ frame->views[0].keypoints[left_keypoint].pt };
    const std::vector<cv::Point2f> distorted_right{ frame->views[1].keypoints[right_keypoint].pt };

    std::vector<cv::Point2f> normalized_left;
    std::vector<cv::Point2f> normalized_right;

    cv::undistortPoints(distorted_left, normalized_left, mLeftCamera->calibration_matrix, mLeftCamera->distortion_coefficients);
    cv::undistortPoints(distorted_right, normalized_right, mLeftCamera->calibration_matrix, mLeftCamera->distortion_coefficients);

    const Eigen::Vector3d C0 = mRig->left_camera_to_rig.translation();
    const Eigen::Matrix3d R0 = mRig->left_camera_to_rig.rotationMatrix();

    const Eigen::Vector3d C1 = mRig->right_camera_to_rig.translation();
    const Eigen::Matrix3d R1 = mRig->right_camera_to_rig.rotationMatrix();

    Eigen::Vector3d D0 = R0 * Eigen::Vector3d{normalized_left.front().x, normalized_left.front().y, 1.0};
    Eigen::Vector3d D1 = R1 * Eigen::Vector3d{normalized_right.front().x, normalized_right.front().y, 1.0};
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

