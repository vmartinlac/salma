#pragma once

#include "StereoRigCalibrationData.h"
#include "SLAMConfiguration.h"
#include "SLAMDataStructures.h"

class SLAMContext
{
public:

    StereoRigCalibrationDataPtr calibration;
    SLAMConfigurationPtr configuration;
    SLAMReconstructionPtr reconstruction;
};

typedef std::shared_ptr<SLAMContext> SLAMContextPtr;

