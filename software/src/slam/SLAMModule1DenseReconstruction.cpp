#include <fstream>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModule1DenseReconstruction.h"
#include "ElasIntf.h"
#include <opencv2/calib3d.hpp>

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

SLAMModuleResult SLAMModule1DenseReconstruction::operator()()
{
    std::cout << "   DENSE RECONSTRUCTION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    cv::Mat original_left = frame->views[0].image;
    cv::Mat original_right = frame->views[1].image;

    cv::Mat rectified_left;
    cv::Mat rectified_right;
    cv::remap( original_left, rectified_left, mRectification.cameras[0].map0, mRectification.cameras[0].map1, cv::INTER_LINEAR );
    cv::remap( original_right, rectified_right, mRectification.cameras[1].map0, mRectification.cameras[1].map1, cv::INTER_LINEAR );

    cv::Mat grey_left;
    cv::Mat grey_right;
    cv::cvtColor(rectified_left, grey_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rectified_right, grey_right, cv::COLOR_BGR2GRAY);

    if( context()->configuration->dense_reconstruction.debug )
    {
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_left.png", rectified_left);
        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_rectified_right.png", rectified_right);
    }

    cv::Mat disparity;
    computeDisparity(grey_left, grey_right, disparity);

    if( disparity.type() != CV_32FC1 ) throw std::runtime_error("incorrect disparity cv::Mat type!");

    if( context()->configuration->dense_reconstruction.debug )
    {
        cv::Mat tmp = disparity.clone();

        double mini = 0.0;
        double maxi = 1.0;
        cv::minMaxLoc(tmp, &mini, &maxi);

        tmp = (tmp - mini) * (255.0 / (maxi - mini));

        context()->debug->saveImage(frame->id, "DENSERECONSTRUCTION_disparity.png", tmp);
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
            if( disparity.at<float>(i, j) > 0.5 )
            {
                const cv::Vec3b colfrom = rectified_left.at<cv::Vec3b>(i, j);

                SLAMColoredPoint cpt;

                const cv::Point3f X = reconstruction.at<cv::Point3f>(i, j);

                const Eigen::Vector3d Y(X.x, X.y, X.z);

                const Eigen::Vector3d Z = mStereoRig->cameras[0].camera_to_rig * Y;

                cpt.point.x = static_cast<float>( Z.x() );
                cpt.point.y = static_cast<float>( Z.y() );
                cpt.point.z = static_cast<float>( Z.z() );
                cpt.color[0] = static_cast<float>(colfrom[0]) / 255.0;
                cpt.color[1] = static_cast<float>(colfrom[1]) / 255.0;
                cpt.color[2] = static_cast<float>(colfrom[2]) / 255.0;

                frame->dense_cloud.push_back(cpt);
            }
        }
    }

    std::cout << "      Size of point cloud: " << frame->dense_cloud.size() << std::endl;

    if( context()->configuration->dense_reconstruction.debug )
    {
        context()->debug->savePointCloud(frame->id, "DENSERECONSTRUCTION_cloud.txt", frame->dense_cloud);
    }

    return SLAMModuleResult(true, SLAM_MODULE1_RECTIFICATION);
}

void SLAMModule1DenseReconstruction::computeDisparity(
    const cv::Mat& rectified_left,
    const cv::Mat& rectified_right,
    cv::Mat& disparity)
{
    cv::Ptr<cv::StereoMatcher> matcher = ElasIntf::create();
    matcher->compute(rectified_left, rectified_right, disparity);

/*
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
            mStereoRig->cameras[0].image_size.width,
            mStereoRig->cameras[0].image_size.height,
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

    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(0, 16, 81);
    //matcher->compute(tmpl, tmpr, disparity);
    matcher->compute(rectified_left, rectified_right, disparity);

#endif
    */
}

