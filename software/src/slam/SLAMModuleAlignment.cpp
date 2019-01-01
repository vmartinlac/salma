#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModuleAlignment.h"

SLAMModuleAlignment::SLAMModuleAlignment(SLAMContextPtr con) : SLAMModule(con)
{
    mSolver.reset(new MVPnP::SolverRANSACLM());
    mSolver->setInlierRate( con->configuration->alignment_ransac_inlier_rate );
    mSolver->setInlierThreshold( con->configuration->alignment_ransac_inlier_threshold );
}

SLAMModuleAlignment::~SLAMModuleAlignment()
{
}

void SLAMModuleAlignment::operator()()
{
    StereoRigCalibrationDataPtr rig = context()->calibration;
    CameraCalibrationDataPtr left_camera = rig->cameras[0].calibration;
    CameraCalibrationDataPtr right_camera = rig->cameras[1].calibration;

/*
    if( frames.empty() ) throw std::runtime_error("internal error");

    if( frames.size() >= 2 )
    {
        std::array<FramePtr,2> lastframes;
        std::copy_n(frames.begin(), 2, lastframes.begin());

        FramePtr current_frame = lastframes[0];
        FramePtr previous_frame = lastframes[1];

        std::vector<MVPnP::View> views(2);

        views[0].calibration_matrix = mLeftCamera->calibration_matrix;
        views[0].distortion_coefficients = mLeftCamera->distortion_coefficients;
        views[0].rig_to_camera = mRig->left_camera_to_rig.inverse();

        views[1].calibration_matrix = mRightCamera->calibration_matrix;
        views[1].distortion_coefficients = mRightCamera->distortion_coefficients;
        views[1].rig_to_camera = mRig->right_camera_to_rig.inverse();

        for(int i=0; i<2; i++)
        {
            for( Projection& p : current_frame->views[i].projections )
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

        current_frame->frame_to_world = previous_frame->frame_to_world;

        const bool ret = mSolver->run(views, current_frame->frame_to_world, true, inliers);

        if(ret)
        {
            if( inliers.size() != 2 ) throw std::runtime_error("internal error");

            for(int i=0; i<2; i++)
            {
                int j = 0;
                while( j < current_frame->views[i].projections.size() )
                {
                    if( inliers[i][j] )
                    {
                        j++;
                    }
                    else
                    {
                        current_frame->views[i].projections[j] = current_frame->views[i].projections.back();
                        current_frame->views[i].projections.pop_back();
                    }
                }
            }

            current_frame->aligned_wrt_previous_frame = true;
        }
        else
        {
            current_frame->frame_to_world = Sophus::SE3d();
            current_frame->aligned_wrt_previous_frame = false;
            current_frame->views[0].projections.clear();
            current_frame->views[1].projections.clear();
        }
    }
    else
    {
        frames.front()->frame_to_world = Sophus::SE3d();
        frames.front()->aligned_wrt_previous_frame = false;
    }
*/
}

