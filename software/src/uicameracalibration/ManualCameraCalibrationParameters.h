#pragma once

#include <memory>
#include <QString>
#include "RecordingHeader.h"

class ManualCameraCalibrationParameters
{
public:

    ManualCameraCalibrationParameters();

    QString name;
    RecordingHeaderPtr recording;
    double scale;
};

typedef std::shared_ptr<ManualCameraCalibrationParameters> ManualCameraCalibrationParametersPtr;

