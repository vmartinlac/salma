
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriangulation : public SLAMModule
{
public:

    SLAMModuleTriangulation(SLAMProjectPtr project);

    void run(FrameList& frames) override;

    int getNumberOfNewMapPoints();

protected:

    MapPointPtr triangulate(FramePtr frame, int left_keypoint, int right_keypoint);

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
    int mInitialLifeTime;
    bool mUseLindstrom;

    int mNumberOfNewMapPoints;
};

typedef std::shared_ptr<SLAMModuleTriangulation> SLAMModuleTriangulationPtr;

