#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModuleAlignment.h"

SLAMModuleAlignment::SLAMModuleAlignment(SLAMProjectPtr project) : SLAMModule(project)
{
    mLeftCamera = project->getLeftCameraCalibration();
    mRightCamera = project->getRightCameraCalibration();
    mRig = project->getStereoRigCalibration();

    //mMaxNumberOfPreviousFrames = project->getParameterInteger("max_number_of_previous_frames", 5);

    mSolver.reset(MVPnP::Solver::create());
}

void SLAMModuleAlignment::run(FramePtr frame)
{
/*
{
    std::vector<MVPnP::View> views(2);

    views[0].calibration_matrix = mLeftCamera->calibration_matrix;
    views[0].distortion_coefficients = mLeftCamera->distortion_coefficients;
    views[0].rig_to_camera = mRig->left_camera_to_rig.inverse();

    views[1].calibration_matrix = mRightCamera->calibration_matrix;
    views[1].distortion_coefficients = mRightCamera->distortion_coefficients;
    views[1].rig_to_camera = mRig->right_camera_to_rig.inverse();

    std::default_random_engine e;
    std::normal_distribution<double> disx(0.0, 1.0);
    std::normal_distribution<double> disy(0.0, 1.0);
    std::normal_distribution<double> disz(100.0, 1.0);

    std::vector<cv::Point3f> pts;
    for(int i=0; i<30; i++)
    {
        const double x = disx(e);
        const double y = disy(e);
        const double z = disz(e);
        pts.push_back(cv::Point3f(x,y,z));
    }

    for(int i=0; i<2; i++)
    {
        auto tmp = 
        Eigen::Vector3d rvec2;
        Eigen::Vector3d tvec2;

        cv::Mat rvec;
        cv::Mat tvec;
        cv::eigen2cv(rvec2, rvec);
        cv::eigen2cv(tvec2, tvec);

        views[i].points = pts;
        cv::projectPoints(pts, rvec, tvec, views[i].calibration_matrix, views[i].distortion_coefficients, views[i].projections);
    }
}
*/
    if( frame->previous_frame )
    {
        std::vector<MVPnP::View> views(2);

        views[0].calibration_matrix = mLeftCamera->calibration_matrix;
        views[0].distortion_coefficients = mLeftCamera->distortion_coefficients;
        views[0].rig_to_camera = mRig->left_camera_to_rig.inverse();

        views[1].calibration_matrix = mRightCamera->calibration_matrix;
        views[1].distortion_coefficients = mRightCamera->distortion_coefficients;
        views[1].rig_to_camera = mRig->right_camera_to_rig.inverse();

        for(int i=0; i<2; i++)
        {
            auto fn = [] (int count, Track& t)
            {
                return (bool(t.mappoint)) ? count+1 : count;
            };

            const int num_points = std::accumulate(
                frame->views[i].tracks.begin(),
                frame->views[i].tracks.end(),
                0,
                fn);

            views[i].points.resize(num_points);
            views[i].projections.resize(num_points);

            int j = 0;
            int k = 0;

            while( k < num_points && j<frame->views[i].keypoints.size() )
            {
                if( frame->views[i].tracks[j].mappoint )
                {
                    const Eigen::Vector3d pt = frame->views[i].tracks[j].mappoint->position;

                    views[i].points[k].x = pt.x();
                    views[i].points[k].y = pt.y();
                    views[i].points[k].z = pt.z();

                    views[i].projections[k] = frame->views[i].keypoints[j].pt;

                    k++;
                }

                j++;
            }

            if( k != num_points) throw std::logic_error("internal error");
        }

        Sophus::SE3d frame_to_world = frame->previous_frame->frame_to_world;
        std::vector< std::vector<bool> > inliers;
        const bool ret = mSolver->run(views, frame_to_world, inliers);

        if(ret)
        {
            frame->frame_to_world = frame_to_world;

            if( inliers.size() != 2 ) throw std::runtime_error("internal error");

            // TODO: remove outliers mappoints!
        }
        else
        {
            // TODO: remove all mappoints and mark frame as not aligned.
        }
    }
}

