
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriangulation : public SLAMModule
{
public:

    SLAMModuleTriangulation(SLAMContextPtr con);
    ~SLAMModuleTriangulation() override;

    bool init() override;
    void operator()() override;

protected:

    SLAMMapPointPtr triangulate(SLAMFramePtr frame, int left_keypoint, int right_keypoint);

    bool triangulateInRigFrame(const cv::Point2f& left_proj, const cv::Point2f& right_proj, Eigen::Vector3d& point, Eigen::Matrix3d& covariance);

    void correctWithLindstrom( Eigen::Vector3d& normalized_left, Eigen::Vector3d& normalized_right );

protected:

    Eigen::Matrix3d mEssentialMatrix;
    Eigen::Matrix2d mEssentialMatrixTilde;
    Eigen::Matrix<double, 2, 3> mS;

    CameraCalibrationDataPtr mLeftCamera;
    CameraCalibrationDataPtr mRightCamera;
    StereoRigCalibrationDataPtr mRig;
    double mMinAngleBetweenRays;
    double mPerpendicularMaxLength;
    bool mCheckPerpendicularLength;
    double mMaxReprojectionError;
    double mMinDistanceToCamera;
    int mInitialLifeTime;
    bool mUseLindstrom;
    double mSigmaProjLeft;
    double mSigmaProjRight;
};

