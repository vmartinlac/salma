#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <utility>
#include <vector>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"

class StereoMatcher
{
public:

    StereoMatcher();

    void setLeftCameraCalibration( CameraCalibrationDataPtr calib );
    void setRightCameraCalibration( CameraCalibrationDataPtr calib );
    void setStereoRigCalibration( StereoRigCalibrationDataPtr calib );

    void setCheckLowe(bool value);
    void setCheckSymmetry(bool value);
    void setCheckEpipolar(bool value);
    void setCheckOctave(bool value);

    void setEpipolarThreshold(double value);
    void setLoweRatio(double value);

    void match(FramePtr frame, std::vector< std::pair<int,int> >& matches);

protected:

    int matchKeyPoint(FramePtr frame, int view, int i, bool check_symmetry);

protected:

    bool mCheckSymmetry;
    bool mCheckLowe;
    bool mCheckEpipolar;
    bool mCheckOctave;
    double mEpipolarThreshold;
    double mLoweRatio;

    CameraCalibrationDataPtr mCameraCalibration[2];
    StereoRigCalibrationDataPtr mStereoRigCalibration;

    Eigen::Matrix3d mFundamentalMatrices[2];
    std::vector<cv::Point2f> mUndistortedPoints[2];
};

