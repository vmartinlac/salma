#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModuleDenseReconstruction.h"

//#define DEBUG_SHOW_RECTIFIED_IMAGES
//#define DEBUG_SAVE_RECTIFIED_IMAGES

SLAMModuleDenseReconstruction::SLAMModuleDenseReconstruction(SLAMContextPtr con) : SLAMModule(con)
{
    mCameras[0] = con->calibration->cameras[0].calibration;
    mCameras[1] = con->calibration->cameras[1].calibration;
    mStereoRig = con->calibration;
}

SLAMModuleDenseReconstruction::~SLAMModuleDenseReconstruction()
{
}

bool SLAMModuleDenseReconstruction::init()
{
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

    return true;
}

void SLAMModuleDenseReconstruction::operator()()
{
    /*
    if( frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = frames.front();

    cv::Mat rectified_left;
    cv::Mat rectified_right;
    cv::remap( frame->views[0].image, rectified_left, mRectification.cameras[0].map0, mRectification.cameras[0].map1, cv::INTER_LINEAR );
    cv::remap( frame->views[1].image, rectified_right, mRectification.cameras[1].map0, mRectification.cameras[1].map1, cv::INTER_LINEAR );

#ifdef DEBUG_SAVE_RECTIFIED_IMAGES
    cv::imwrite("left_"+std::to_string(frame->id)+".png", rectified_left);
    cv::imwrite("right_"+std::to_string(frame->id)+".png", rectified_right);
#endif

#ifdef DEBUG_SHOW_RECTIFIED_IMAGES
    Debug::stereoimshow( rectified_left, rectified_right ); 
#endif
*/
}

