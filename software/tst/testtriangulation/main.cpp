#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include "SLAMModule1Triangulation.h"
#include "SLAMDebug.h"
#include "SLAMConfiguration.h"
#include "SLAMContext.h"

int main(int num_args, char** args)
{
    cv::Mat calibration_matrix = cv::Mat::zeros(3,3,CV_64F);
    calibration_matrix.at<double>(0,0) = 2200.0;
    calibration_matrix.at<double>(0,2) = 650.0;
    calibration_matrix.at<double>(1,1) = 2200.0;
    calibration_matrix.at<double>(1,2) = 460.0;
    calibration_matrix.at<double>(2,2) = 1.0;

    StereoRigCalibrationPtr rig(new StereoRigCalibration());
    rig->cameras[0].calibration_matrix = calibration_matrix;
    rig->cameras[0].camera_to_rig.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
    rig->cameras[1].calibration_matrix = calibration_matrix;
    rig->cameras[1].camera_to_rig.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);

    SLAMContextPtr con(new SLAMContext());

    con->configuration.reset(new SLAMConfiguration());
    con->calibration = rig;
    con->reconstruction.reset(new SLAMReconstruction());
    con->debug.reset(new SLAMDebug(con->configuration));

    SLAMFramePtr frame(new SLAMFrame());
    con->reconstruction->frames.push_back(frame);

    frame->id = 1;
    frame->rank_in_recording = 0;
    frame->timestamp = 0.0;
    frame->pose_covariance.setZero();
    frame->pose_covariance.block<3,3>(0,0) = 0.01*Eigen::Matrix3d::Identity();
    //frame->pose_covariance.block<4,4>(3,3) = 0.01*Eigen::Matrix4d::Identity();
    //frame->pose_covariance(2,2) = 1.0;

    {
        Eigen::Vector3d X;
        X << -2.0, -2.0, 10;

        Eigen::Matrix3d K;
        cv::cv2eigen(calibration_matrix, K);

        for(int i=0; i<2; i++)
        {
            Eigen::Vector3d x = K * ( rig->cameras[i].camera_to_rig.inverse() * frame->frame_to_world.inverse() * X );
            x /= x.z();

            frame->views[i].keypoints.resize(1);
            frame->views[i].keypoints.front().pt.x = x.x();
            frame->views[i].keypoints.front().pt.y = x.y();

            frame->views[i].tracks.resize(1);
        }

        frame->stereo_matches.push_back( std::pair<int,int>(0,0) );
    }

    SLAMModulePtr mod(new SLAMModule1Triangulation(con));

    mod->init();
    (*mod)();

    std::cout << frame->views[0].tracks.front().mappoint->position.transpose() << std::endl;
    std::cout << frame->views[0].tracks.front().mappoint->position_covariance << std::endl;

    return 0;
}

