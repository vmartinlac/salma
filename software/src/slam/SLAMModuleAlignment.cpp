#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModuleAlignment.h"

SLAMModuleAlignment::SLAMModuleAlignment(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleAlignment::~SLAMModuleAlignment()
{
}

bool SLAMModuleAlignment::init()
{
    SLAMContextPtr con = context();

    mSolver.reset(new MVPnP::SolverRANSACLM());
    mSolver->setInlierRate( con->configuration->alignment_ransac_inlier_rate );
    mSolver->setInlierThreshold( con->configuration->alignment_ransac_inlier_threshold );

    return true;
}

void SLAMModuleAlignment::operator()()
{
    StereoRigCalibrationDataPtr rig = context()->calibration;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    const int N_frames = reconstr->frames.size();

    if( N_frames >= 2 )
    {
        SLAMFramePtr current_frame = reconstr->frames[N_frames-1];
        SLAMFramePtr previous_frame = reconstr->frames[N_frames-2];

        std::vector<MVPnP::View> views(2);

        std::vector<int> projections[2];

        for(int i=0; i<2; i++)
        {
            views[i].calibration_matrix = rig->cameras[i].calibration->calibration_matrix;
            views[i].distortion_coefficients = rig->cameras[i].calibration->distortion_coefficients;
            views[i].rig_to_camera = rig->cameras[i].camera_to_rig.inverse();

            for(int j=0; j<current_frame->views[i].tracks.size(); j++)
            {
                if( current_frame->views[i].tracks[j].projection_type == SLAM_PROJECTION_TRACKED )
                {
                    SLAMMapPointPtr mp = current_frame->views[i].tracks[j].projection_mappoint;

                    projections[i].push_back(j);

                    cv::Point3f world;
                    world.x = mp->position.x();
                    world.y = mp->position.y();
                    world.z = mp->position.z();

                    views[i].projections.push_back( current_frame->views[i].keypoints[j].pt );
                    views[i].points.push_back( world );
                }
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
                if( inliers[i].size() != projections[i].size() ) throw std::runtime_error("internal error");

                for(int j=0; j<inliers[i].size(); j++)
                {
                    if(inliers[i][j] == false)
                    {
                        const int keypoint = projections[i][j];
                        current_frame->views[i].tracks[keypoint].projection_type = SLAM_PROJECTION_NONE;
                        current_frame->views[i].tracks[keypoint].projection_mappoint.reset();
                    }
                }
            }

            current_frame->aligned_wrt_previous_frame = true;
        }
        else
        {
            current_frame->frame_to_world = Sophus::SE3d();
            current_frame->aligned_wrt_previous_frame = false;

            for(int i=0; i<2; i++)
            {
                for(int j=0; j<current_frame->views[i].tracks.size(); j++)
                {
                    current_frame->views[i].tracks[j].projection_type = SLAM_PROJECTION_NONE;
                    current_frame->views[i].tracks[j].projection_mappoint.reset();
                }
            }
        }
    }
    else if(N_frames == 1)
    {
        reconstr->frames.front()->frame_to_world = Sophus::SE3d();
        reconstr->frames.front()->aligned_wrt_previous_frame = false;
    }
    else
    {
        throw std::runtime_error("internal error");
    }
}

