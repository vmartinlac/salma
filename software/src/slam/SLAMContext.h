#pragma once

#include "StereoRigCalibrationData.h"
#include "SLAMConfiguration.h"
#include "SLAMDataStructures.h"
#include "SLAMDebug.h"

class SLAMContext
{
public:

    SLAMContext();

    StereoRigCalibrationDataPtr calibration;
    SLAMConfigurationPtr configuration;
    SLAMReconstructionPtr reconstruction;
    SLAMDebugPtr debug;
    int num_mappoints;
};

typedef std::shared_ptr<SLAMContext> SLAMContextPtr;

