#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModule1Rectification.h"

SLAMModule1Rectification::SLAMModule1Rectification(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_RECTIFICATION, con)
{
}

SLAMModule1Rectification::~SLAMModule1Rectification()
{
}

bool SLAMModule1Rectification::init()
{
    SLAMContextPtr con = context();

    mStereoRig = con->calibration;

    const Sophus::SE3d left_to_right = mStereoRig->cameras[1].camera_to_rig.inverse() * mStereoRig->cameras[0].camera_to_rig;

    cv::eigen2cv( left_to_right.rotationMatrix(), mRectification.R );
    cv::eigen2cv( left_to_right.translation(), mRectification.T );

    cv::stereoRectify(
        mStereoRig->cameras[0].calibration_matrix,
        mStereoRig->cameras[0].distortion_coefficients,
        mStereoRig->cameras[1].calibration_matrix,
        mStereoRig->cameras[1].distortion_coefficients,
        mStereoRig->cameras[0].image_size,
        mRectification.R,
        mRectification.T,
        mRectification.cameras[0].R,
        mRectification.cameras[1].R,
        mRectification.cameras[0].P,
        mRectification.cameras[1].P,
        mRectification.Q,
        cv::CALIB_ZERO_DISPARITY,
        0.0);

    for(int i=0; i<2; i++)
    {
        cv::initUndistortRectifyMap(
            mStereoRig->cameras[i].calibration_matrix,
            mStereoRig->cameras[i].distortion_coefficients,
            cv::Mat(), //mRectification.cameras[i].R,
            mRectification.cameras[i].P,
            mStereoRig->cameras[i].image_size,
            CV_32FC1,
            mRectification.cameras[i].map0,
            mRectification.cameras[i].map1 );
    }

    return true;
}

SLAMModuleResult SLAMModule1Rectification::operator()()
{
    std::cout << "   RECTIFICATION" << std::endl;

    /*
    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    cv::Mat original_left = frame->views[0].image;
    cv::Mat original_right = frame->views[1].image;

    cv::Mat rectified_left;
    cv::Mat rectified_right;
    cv::remap( original_left, rectified_left, mRectification.cameras[0].map0, mRectification.cameras[0].map1, cv::INTER_LINEAR );
    cv::remap( original_right, rectified_right, mRectification.cameras[1].map0, mRectification.cameras[1].map1, cv::INTER_LINEAR );

    if( context()->configuration->rectification.debug )
    {
        context()->debug->saveImage(frame->id, "RECTIFICATION_left.png", rectified_left);
        context()->debug->saveImage(frame->id, "RECTIFICATION_right.png", rectified_right);
    }
    */

    /*
    frame->views[0].image = rectified_left;
    frame->views[1].image = rectified_right;
    */

    return SLAMModuleResult(false, SLAM_MODULE1_FEATURES);
}

