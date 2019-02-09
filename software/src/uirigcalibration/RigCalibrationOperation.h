
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <QDir>
#include <QTime>
#include <sophus/se3.hpp>
#include "PathWidget.h"
#include "Tracker.h"
#include "VideoSource.h"
#include "Operation.h"
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"

class RigCalibrationOperation : public Operation
{
public:

    RigCalibrationOperation();

    ~RigCalibrationOperation() override;

    const char* getName() override;

    bool before() override;
    bool step() override;
    void after() override;
    void uiafter(QWidget* parent, Project* project) override;

public:

    std::string mCalibrationName;
    CameraCalibrationDataPtr mLeftCalibrationData;
    CameraCalibrationDataPtr mRightCalibrationData;
    double mTargetCellLength;
    VideoSourcePtr mCamera;
    int mNumberOfPosesForCalibration;
    int mMillisecondsOfTemporisation;

protected:

    void writeOutputText();

    void calibrate();

protected:

    target::Tracker mLeftTracker;
    target::Tracker mRightTracker;
    int mFrameCount;
    QTime mClock;
    QTime mTriggerClock;
    std::vector< std::vector<cv::Point2f> > mLeftImagePoints;
    std::vector< std::vector<cv::Point2f> > mRightImagePoints;
    std::vector< std::vector<cv::Point3f> > mObjectPoints;
    StereoRigCalibrationDataPtr mResult;
};
