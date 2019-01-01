#pragma once

#include <memory>
#include <QDir>
#include "RecordingReader.h"
#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"

class SLAMConfiguration
{
public:

    int features_max_keypoints;
};

typedef std::shared_ptr<SLAMConfiguration> SLAMConfigurationPtr;

class SLAMContext
{
public:

    SLAMContext();
    ~SLAMContext();

    SLAMReconstructionPtr reconstruction;
    StereoRigCalibrationDataPtr calibration;
    RecordingReaderPtr recording;
    SLAMConfigurationPtr configuration;
};

typedef std::shared_ptr<SLAMContext> SLAMContextPtr;

class SLAMEngine
{
};

