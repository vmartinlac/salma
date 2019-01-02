#pragma once

#include "StereoRigCalibrationData.h"
#include "SLAMConfiguration.h"
#include "SLAMDataStructures.h"
#include "SLAMDebug.h"

class SLAMContext
{
public:

    StereoRigCalibrationDataPtr calibration;
    SLAMConfigurationPtr configuration;
    SLAMReconstructionPtr reconstruction;
    SLAMDebugPtr debug;
};

typedef std::shared_ptr<SLAMContext> SLAMContextPtr;

