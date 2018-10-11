#include "Operation.h"

Operation::Operation()
{
}

void Operation::setPorts(
    VideoInputPort* video,
    StatsInputPort* stats)
{
    mVideoPort = video;
    mStatsPort = stats;
}

CameraCalibrationOperation* Operation::cameraCalibration()
{
    return nullptr;
}

StereoRigCalibrationOperation* Operation::stereoRigCalibration()
{
    return nullptr;
}

MonoRecordingOperation* Operation::monoRecording()
{
    return nullptr;
}

StereoRecordingOperation* Operation::stereoRecording()
{
    return nullptr;
}

void Operation::before()
{
}

void Operation::after()
{
}

