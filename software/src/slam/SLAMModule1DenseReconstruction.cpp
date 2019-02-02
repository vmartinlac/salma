#include <fstream>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModule1DenseReconstruction.h"
#if defined(SALMA_WITH_CUDA)
#include "StereoMatcher.h"
#elif defined(SALMA_OPENCV_HAS_CUDA)
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#else
#include <opencv2/stereo.hpp>
#endif

SLAMModule1DenseReconstruction::SLAMModule1DenseReconstruction(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_DENSERECONSTRUCTION, con)
{
}

SLAMModule1DenseReconstruction::~SLAMModule1DenseReconstruction()
{
}

bool SLAMModule1DenseReconstruction::init()
{
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
        mRectification.Q,
        cv::CALIB_ZERO_DISPARITY,
        0.0);

    for(int i=0; i<2; i++)
    {
        cv::initUndistortRectifyMap(
            mCameras[i]->calibration_matrix,
            mCameras[i]->distortion_coefficients,
            cv::Mat(), //mRectification.cameras[i].R,
            mRectification.cameras[i].P,
            mCameras[i]->image_size,
            CV_32FC1,
            mRectification.cameras[i].map0,
            mRectification.cameras[i].map1 );
    }

    return true;
}

SLAMModuleResult SLAMModule1DenseReconstruction::operator()()
{
    std::cout << "   DENSE RECONSTRUCTION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    cv::Mat rectified_left;
    cv::Mat rectified_right;
    cv::remap( frame->views[0].image, rectified_left, mRectification.cameras[0].map0, mRectification.cameras[0].map1, cv::INTER_LINEAR );
    cv::remap( frame->views[1].image, rectified_right, mRectification.cameras[1].map0, mRectification.cameras[1].map1, cv::INTER_LINEAR );

    if( context()->configuration->dense_reconstruction.debug )
    {
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_left", rectified_left);
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_right", rectified_right);
    }

    cv::Mat disparity;
    computeDisparity(rectified_left, rectified_right, disparity);

    if( disparity.type() != CV_16SC1 ) throw std::runtime_error("incorrect disparity cv::Mat type!");

    if( context()->configuration->dense_reconstruction.debug )
    {
        cv::Mat tmp;
        disparity.convertTo(tmp, CV_32F);

        double mini = 0.0;
        double maxi = 1.0;
        cv::minMaxLoc(tmp, &mini, &maxi);

        tmp = (tmp - mini) * (255.0 / (maxi - mini));

        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_disparity", tmp);
    }

    cv::Mat reconstruction;
    cv::reprojectImageTo3D(
        disparity,
        reconstruction,
        mRectification.Q,
        false,
        CV_32F);

    if( reconstruction.type() != CV_32FC3 ) throw std::runtime_error("incorrect disparity cv::Mat type!");

    frame->dense_cloud.clear();

    for(int i=0; i<reconstruction.rows; i++)
    {
        for(int j=0; j<reconstruction.cols; j++)
        {
            if( disparity.at<int16_t>(i, j) != 0 )
            {
                const cv::Point3f pt = reconstruction.at<cv::Point3f>(i, j);

                frame->dense_cloud.push_back(pt);
            }
        }
    }

    std::cout << "      Size of point cloud: " << frame->dense_cloud.size() << std::endl;

    return SLAMModuleResult(true, SLAM_MODULE1_FEATURES);
}

void SLAMModule1DenseReconstruction::computeDisparity(
    const cv::Mat& rectified_left,
    const cv::Mat& rectified_right,
    cv::Mat& disparity)
{

#if defined(SALMA_WITH_CUDA)

    std::cout << "      Computing disparity with salma belief propagation stereo matcher." << std::endl;

    cv::cuda::GpuMat d_rectified_left;
    cv::cuda::GpuMat d_rectified_right;

    d_rectified_left.upload(rectified_left);
    d_rectified_right.upload(rectified_right);

    cv::cuda::GpuMat d_disparity;

    cv::Ptr<StereoMatcher> matcher = StereoMatcher::create();

    d_disparity.download(disparity);

#elif defined(SALMA_OPENCV_HAS_CUDA)

    std::cout << "      Computing disparity with OpenCV CUDA Belief Propagation stereo matcher." << std::endl;

    cv::Ptr<cv::cuda::StereoBeliefPropagation> matcher;

    {
        int ndisp = 0;
        int iters = 0;
        int levels = 0;

        cv::cuda::StereoBeliefPropagation::estimateRecommendedParams(
            mCameras[0]->image_size.width,
            mCameras[0]->image_size.height,
            ndisp,
            iters,
            levels);

        matcher = cv::cuda::createStereoBeliefPropagation(ndisp, iters, levels);
    }

    cv::cuda::GpuMat d_rectified_left;
    cv::cuda::GpuMat d_rectified_right;

    d_rectified_left.upload(rectified_left);
    d_rectified_right.upload(rectified_right);

    cv::cuda::GpuMat d_disparity;

    matcher->compute(d_rectified_left, d_rectified_right, d_disparity);

    d_disparity.download(disparity);

#else

    std::cout << "      Computing disparity with OpenCV SGBM stereo matcher." << std::endl;

    /*
    cv::Mat tmpl;
    cv::Mat tmpr;
    cv::resize(rectified_left, tmpl, cv::Size(), 0.25, 0.25);
    cv::resize(rectified_right, tmpr, cv::Size(), 0.25, 0.25);
    */

    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(0, 16, 81);
    //matcher->compute(tmpl, tmpr, disparity);
    matcher->compute(rectified_left, rectified_right, disparity);

#endif

}

