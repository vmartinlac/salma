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
    if( frame->previous_frame )
    {
        std::vector<MVPnP::View> views(2);

        views[0].calibration_matrix = mLeftCamera->calibration_matrix;
        views[0].distortion_coefficients = mLeftCamera->distortion_coefficients;
        views[0].rig_to_camera = mRig->left_camera_to_rig.inverse();

        views[1].calibration_matrix = mRightCamera->calibration_matrix;
        views[1].distortion_coefficients = mRightCamera->distortion_coefficients;
        views[1].rig_to_camera = mRig->right_camera_to_rig.inverse();

        const Sophus::SE3d world_to_previous_frame = frame->previous_frame->frame_to_world.inverse();

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
                    Eigen::Vector3d pt = world_to_previous_frame * frame->views[i].tracks[j].mappoint->position;

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

        Sophus::SE3d frame_to_previous_frame;
        std::vector< std::vector<bool> > inliers;
        const bool ret = mSolver->run(views, frame_to_previous_frame, inliers);

        if(ret)
        {
            frame->frame_to_world = frame->previous_frame->frame_to_world * frame_to_previous_frame;

            if( inliers.size() != 2 ) throw std::runtime_error("internal error");

            // TODO: remove outliers mappoints!
        }
        else
        {
            // TODO: remove all mappoints and mark frame as not aligned.
        }
    }
}

