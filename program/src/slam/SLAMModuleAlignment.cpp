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

        for(int i=0; i<2; i++)
        {
            for( Projection& p : frame->views[i].projections )
            {
                cv::Point3f world;
                world.x = p.mappoint->position.x();
                world.y = p.mappoint->position.y();
                world.z = p.mappoint->position.z();

                views[i].projections.push_back( p.point );
                views[i].points.push_back( world );
            }
        }

        std::vector< std::vector<bool> > inliers;

        frame->frame_to_world = frame->previous_frame->frame_to_world;

        const bool ret = mSolver->run(views, frame->frame_to_world, true, inliers);

        if(ret)
        {
            if( inliers.size() != 2 ) throw std::runtime_error("internal error");

            for(int i=0; i<2; i++)
            {
                int j = 0;
                while( j < frame->views[i].projections.size() )
                {
                    if( inliers[i][j] )
                    {
                        j++;
                    }
                    else
                    {
                        frame->views[i].projections[j] = frame->views[i].projections.back();
                        frame->views[i].projections.pop_back();
                    }
                }
            }

            frame->aligned_wrt_previous_frame = true;
        }
        else
        {
            frame->frame_to_world = Sophus::SE3d();
            frame->aligned_wrt_previous_frame = false;
            frame->views[0].projections.clear();
            frame->views[1].projections.clear();
        }
    }
    else
    {
        frame->frame_to_world = Sophus::SE3d();
        frame->aligned_wrt_previous_frame = false;
    }
}

