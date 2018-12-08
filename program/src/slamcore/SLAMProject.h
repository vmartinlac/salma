#pragma once

#include <memory>
#include <QJsonObject>
#include <QDir>
#include "VideoSource.h"
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"

class SLAMProject
{
public:

    SLAMProject();
    ~SLAMProject();

    bool load(const char* path);

    CameraCalibrationDataPtr getLeftCameraCalibration();
    CameraCalibrationDataPtr getRightCameraCalibration();
    StereoRigCalibrationDataPtr getStereoRigCalibration();

    VideoSourcePtr getVideo();

    bool getParameterBoolean(const char* name, bool default_value);
    int getParameterInteger(const char* name, int default_value);
    double getParameterReal(const char* name, double default_value);

    bool exportReconstruction(FramePtr last_frame, const std::string& name);

protected:

    QDir mDir;
    CameraCalibrationDataPtr mLeftCamera;
    CameraCalibrationDataPtr mRightCamera;
    StereoRigCalibrationDataPtr mStereoRig;
    VideoSourcePtr mVideo;
    QJsonObject mParameters;
};

typedef std::shared_ptr<SLAMProject> SLAMProjectPtr;

