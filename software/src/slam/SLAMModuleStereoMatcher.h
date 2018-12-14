#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <utility>
#include <vector>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleStereoMatcher : public SLAMModule
{
public:

    SLAMModuleStereoMatcher(SLAMProjectPtr project);
    ~SLAMModuleStereoMatcher() override;

    void run(FrameList& frames) override;

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

typedef std::shared_ptr<SLAMModuleStereoMatcher> SLAMModuleStereoMatcherPtr;

