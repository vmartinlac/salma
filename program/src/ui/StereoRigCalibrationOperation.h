
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <QDir>
#include <QTime>
#include <sophus/se3.hpp>
#include "Tracker.h"
#include "VideoSource.h"
#include "Operation.h"

class StereoRigCalibrationOperation : public Operation
{
public:

    StereoRigCalibrationOperation();

    ~StereoRigCalibrationOperation() override;

    bool before() override;
    bool step() override;
    void after() override;

public:

    VideoSourcePtr mCamera;
    int mNumberOfCalibrationPoses;
    int mMillisecondsOfTemporisation;
    std::string mOutputPath;

protected:

    void writeOutputText();

protected:

    target::Tracker mLeftTracker;
    target::Tracker mRightTracker;
    int mFrameCount;
    QTime mClock;
    std::vector< Sophus::SE3<double> > mPoses;
};

