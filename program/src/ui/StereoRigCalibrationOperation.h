
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

class StereoRigCalibrationOperation : public Operation
{
public:

    StereoRigCalibrationOperation();

    ~StereoRigCalibrationOperation() override;

    bool before() override;
    bool step() override;
    void after() override;

public:

    CameraCalibrationData mLeftCalibrationData;
    CameraCalibrationData mRightCalibrationData;
    VideoSourcePtr mCamera;
    int mNumberOfPosesForCalibration;
    int mMillisecondsOfTemporisation;
    std::string mOutputPath;

protected:

    void writeOutputText();

    void calibrate();

    static void convertPoseFromOpenCVToSophus(
        const cv::Mat& rodrigues,
        const cv::Mat& t,
        Sophus::SE3d& camera_to_object);

    static bool computePose(
        target::Tracker& tracker,
        CameraCalibrationData& calibration,
        Sophus::SE3d& camera_to_target);

protected:

    target::Tracker mLeftTracker;
    target::Tracker mRightTracker;
    int mFrameCount;
    QTime mClock;
    std::vector<Sophus::SE3d> mPoses;
};

