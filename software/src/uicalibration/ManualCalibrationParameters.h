#pragma once

#include <memory>
#include <QString>
#include "RecordingHeader.h"

class ManualCalibrationParameters
{
public:

    ManualCalibrationParameters();

    QString name;
    RecordingHeaderPtr recording;
};

typedef std::shared_ptr<ManualCalibrationParameters> ManualCalibrationParametersPtr;

