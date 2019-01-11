
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

    void correctWithLindstrom( Eigen::Vector3d& normalized_left, Eigen::Vector3d& normalized_right );

    void computePositionCovariance(SLAMFramePtr frame, const Eigen::Vector3d& normalized_left, const Eigen::Vector3d& normalized_right, Eigen::Matrix<double, 3, 10>& cov);

    static void computeJacobianOfTriangulation(const Eigen::Vector3d& O1, const Eigen::Vector3d& D1, const Eigen::Vector3d& O2, const Eigen::Vector3d& D2, Eigen::Matrix<double, 3, 12>& J);

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
};

