#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModuleAlignment.h"

SLAMModuleAlignment::SLAMModuleAlignment(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_ALIGNMENT, con)
{
}

SLAMModuleAlignment::~SLAMModuleAlignment()
{
}

bool SLAMModuleAlignment::init()
{
    SLAMContextPtr con = context();

    mSolver.reset(new MVPnP::SolverRANSACLM());
    mSolver->setInlierRate( con->configuration->alignment.ransac_inlier_rate );
    mSolver->setInlierThreshold( con->configuration->alignment.ransac_inlier_threshold );

    return true;
}

SLAMModuleResult SLAMModuleAlignment::operator()()
{
    std::cout << "   ALIGNMENT" << std::endl;

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

            SLAMView& v = current_frame->views[i];
            const int N_keypoints = v.keypoints.size();

            for(int j=0; j<N_keypoints; j++)
            {
                if( v.tracks[j].mappoint )
                {
                    projections[i].push_back(j);

                    cv::Point3f world;
                    world.x = v.tracks[j].mappoint->position.x();
                    world.y = v.tracks[j].mappoint->position.y();
                    world.z = v.tracks[j].mappoint->position.z();

                    views[i].projections.push_back( v.keypoints[j].pt );
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

            int num_outliers = 0;
            int num_inliers_and_outliers = 0;

            for(int i=0; i<2; i++)
            {
                if( inliers[i].size() != projections[i].size() ) throw std::runtime_error("internal error");

                for(int j=0; j<inliers[i].size(); j++)
                {
                    const int keypoint = projections[i][j];

                    SLAMMapPointPtr& mp = current_frame->views[i].tracks[keypoint].mappoint;

                    if(inliers[i][j])
                    {
                        mp->num_inlier_verdicts++;
                    }
                    else
                    {
                        mp->num_outlier_verdicts++;
                        mp.reset();
                        num_outliers++;
                    }

                    num_inliers_and_outliers++;
                }
            }

            current_frame->aligned_wrt_previous_frame = true;

            std::cout << "      Number of outliers: " << num_outliers << " ( " << round(100.0*num_outliers/num_inliers_and_outliers) << " % )" << std::endl;
        }
        else
        {
            current_frame->frame_to_world = Sophus::SE3d();
            current_frame->aligned_wrt_previous_frame = false;

            for(int i=0; i<2; i++)
            {
                for(int j=0; j<current_frame->views[i].tracks.size(); j++)
                {
                    current_frame->views[i].tracks[j].mappoint.reset();
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

    std::cout << "      Alignment status: " << ( (reconstr->frames.back()->aligned_wrt_previous_frame) ? "ALIGNED" : "NOT ALIGNED" ) << std::endl;
    std::cout << "      Position: " << reconstr->frames.back()->frame_to_world.translation().transpose() << std::endl;
    std::cout << "      Attitude: " << reconstr->frames.back()->frame_to_world.unit_quaternion().coeffs().transpose() << std::endl;

    return SLAMModuleResult(false, SLAM_MODULE1_LBA);
}

