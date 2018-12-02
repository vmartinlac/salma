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

    mSolver.reset(new MVPnP::SolverRANSACLM());

    mSolver->setInlierRate( project->getParameterReal("alignment_ransac_inlier_rate", 0.8) );
    mSolver->setInlierThreshold( project->getParameterReal("alignment_ransac_inlier_threshold", 8.0) );
}

void SLAMModuleAlignment::run(FramePtr frame)
{
    if( frame->previous_frame )
    {
        std::vector<MVPnP::View> views(2);

        views[0].calibration_matrix = mLeftCamera->calibration_matrix;
        views[0].distortion_coefficients = mLeftCamera->distortion_coefficients;
        views[0].rig_to_camera = mRig->left_camera_to_rig.inverse();

        views[1].calibration_matrix = mRightCamera->calibration_matrix;
        views[1].distortion_coefficients = mRightCamera->distortion_coefficients;
        views[1].rig_to_camera = mRig->right_camera_to_rig.inverse();

        auto counter = [] (int count, Track& t)
        {
            return (bool(t.mappoint)) ? count+1 : count;
        };

        for(int i=0; i<2; i++)
        {
            const int num_points = std::accumulate(
                frame->views[i].tracks.begin(),
                frame->views[i].tracks.end(),
                0,
                counter);

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

        frame->frame_to_world = frame->previous_frame->frame_to_world;

        std::vector< std::vector<bool> > inliers;

        const bool ret = mSolver->run(views, frame->frame_to_world, true, inliers);

        frame->aligned_wrt_previous_frame = ret;

        if(ret)
        {
            if( inliers.size() != 2 ) throw std::runtime_error("internal error");

            for(int i=0; i<2; i++)
            {
                for(int j=0; j<frame->views[i].keypoints.size(); j++)
                {
                    if( inliers[i][j] == false )
                    {
                        frame->views[i].tracks[j].mappoint.reset();
                    }
                }
            }
        }
        else
        {
            frame->frame_to_world = Sophus::SE3d();

            for(int i=0; i<2; i++)
            {
                for(int j=0; j<frame->views[i].keypoints.size(); j++)
                {
                    frame->views[i].tracks[j].mappoint.reset();
                }
            }
        }
    }
}

