
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <QDir>
#include <QTime>
#include "Tracker.h"
#include "VideoSource.h"
#include "Operation.h"

class CameraCalibrationOperation : public Operation
{
public:

    CameraCalibrationOperation();

    ~CameraCalibrationOperation() override;

    bool before() override;
    bool step() override;
    void after() override;
    const char* getName() override;

public:

    std::string mOutputPath;
    double mTargetCellLength;
    VideoSourcePtr mCamera;
    int mRequestedSuccessfulFrameCount;
    int mMillisecondsTemporisation;

protected:

    void writeOutputText();

protected:

    target::Tracker mTracker;
    int mFrameCount;
    int mAttemptedFrameCount;
    int mSuccessfulFrameCount;
    std::vector< std::vector<cv::Point3f> > mObjectPoints;
    std::vector< std::vector<cv::Point2f> > mImagePoints;
    cv::Size mImageSize;
    QTime mClock;
};

