#pragma once

#include <memory>
#include "VideoWidget.h"
#include "StatsWidget.h"

class CameraCalibrationOperation;
class StereoRigCalibrationOperation;
class MonoRecordingOperation;
class StereoRecordingOperation;

class Operation
{
public:

    enum OperationName
    {
        OPERATION_CAMERA_CALIBRATION,
        OPERATION_STEREO_CALIBRATION,
        OPERATION_MONO_RECORDING,
        OPERATION_STEREO_RECORDING
    };

public:

    Operation();

    virtual ~Operation() = 0;

    virtual OperationName getOperation() = 0;

    virtual CameraCalibrationOperation* cameraCalibration();
    virtual StereoRigCalibrationOperation* stereoRigCalibration();
    virtual MonoRecordingOperation* monoRecording();
    virtual StereoRecordingOperation* stereoRecording();

    void setPorts( VideoInputPort* video, StatsInputPort* stats );

    virtual void before();
    virtual bool step() = 0;
    virtual void after();

protected:

    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;
};

typedef std::shared_ptr<Operation> OperationPtr;

