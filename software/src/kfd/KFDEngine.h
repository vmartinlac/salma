#pragma once

#include <QThread>
#include "KFDPose.h"
#include "VideoSource.h"
#include "CameraCalibrationData.h"

class KFDEngine : public QThread
{
public:

    KFDEngine(QObject* parent=nullptr);
    ~KFDEngine() override;

    KFDPosePort* posePort();

    void run() override;

public:

    VideoSourcePtr mCamera;
    CameraCalibrationDataPtr mCalibration;
    double mTargetScale;

protected:

    KFDPosePort* mPosePort;
};

