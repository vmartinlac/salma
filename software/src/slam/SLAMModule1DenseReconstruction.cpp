#include <fstream>
#include <Eigen/Eigen>
//#include <opencv2/cudastereo.hpp>
//#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModule1DenseReconstruction.h"

SLAMModule1DenseReconstruction::SLAMModule1DenseReconstruction(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_DENSERECONSTRUCTION, con)
{
}

SLAMModule1DenseReconstruction::~SLAMModule1DenseReconstruction()
{
}

bool SLAMModule1DenseReconstruction::init()
{
    /*
    SLAMContextPtr con = context();

    mCameras[0] = con->calibration->cameras[0].calibration;
    mCameras[1] = con->calibration->cameras[1].calibration;
    mStereoRig = con->calibration;

    const Sophus::SE3d left_to_right = mStereoRig->cameras[1].camera_to_rig.inverse() * mStereoRig->cameras[0].camera_to_rig;

    cv::eigen2cv( left_to_right.rotationMatrix(), mRectification.R );
    cv::eigen2cv( left_to_right.translation(), mRectification.T );

    cv::stereoRectify(
        mCameras[0]->calibration_matrix,
        mCameras[0]->distortion_coefficients,
        mCameras[1]->calibration_matrix,
        mCameras[1]->distortion_coefficients,
        mCameras[0]->image_size,
        mRectification.R,
        mRectification.T,
        mRectification.cameras[0].R,
        mRectification.cameras[1].R,
        mRectification.cameras[0].P,
        mRectification.cameras[1].P,
        mRectification.Q);

    for(int i=0; i<2; i++)
    {
        cv::initUndistortRectifyMap(
            mCameras[i]->calibration_matrix,
            mCameras[i]->distortion_coefficients,
            mRectification.cameras[i].R,
            mRectification.cameras[i].P,
            mCameras[i]->image_size,
            CV_32FC1,
            mRectification.cameras[i].map0,
            mRectification.cameras[i].map1 );
    }
    */

    return true;
}

SLAMModuleResult SLAMModule1DenseReconstruction::operator()()
{
    /*
    std::cout << "   DENSE RECONSTRUCTION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    cv::Mat rectified_left;
    cv::Mat rectified_right;
    cv::remap( frame->views[0].image, rectified_left, mRectification.cameras[0].map0, mRectification.cameras[0].map1, cv::INTER_LINEAR );
    cv::remap( frame->views[1].image, rectified_right, mRectification.cameras[1].map0, mRectification.cameras[1].map1, cv::INTER_LINEAR );

    if( context()->configuration->densereconstruction_debug )
    {
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_left", rectified_left);
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_right", rectified_right);
    }

    //
    cv::Ptr<cv::cuda::StereoBeliefPropagation> matcher;

    {
        int ndisp = 0;
        int iters = 0;
        int levels = 0;

        cv::cuda::StereoBeliefPropagation::estimateRecommendedParams(
            mCameras[0]->image_size.width/8,
            mCameras[0]->image_size.height/8,
            ndisp,
            iters,
            levels);

        matcher = cv::cuda::createStereoBeliefPropagation(ndisp, iters, levels);
    }

    cv::Mat tmpl;
    cv::Mat tmpr;
    cv::resize(rectified_left, tmpl, cv::Size(), 0.125, 0.125);
    cv::resize(rectified_right, tmpr, cv::Size(), 0.125, 0.125);
    rectified_left = tmpl;
    rectified_right = tmpr;

    cv::cuda::GpuMat d_rectified_left;
    cv::cuda::GpuMat d_rectified_right;

    d_rectified_left.upload(rectified_left);
    d_rectified_right.upload(rectified_right);

    cv::cuda::GpuMat d_disparity;

    matcher->compute(d_rectified_left, d_rectified_right, d_disparity);

    cv::Mat disparity;
    d_disparity.download(disparity);

    if( context()->configuration->densereconstruction_debug )
    {
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_tmpdisparity", disparity);
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_tmpleft", rectified_left);
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_tmpright", rectified_right);
    }
    */

    return SLAMModuleResult(true, SLAM_MODULE1_FEATURES);
}

