#pragma once

#include "SLAMDataStructures.h"
#include "StereoRigCalibrationData.h"
#include "SLAMConfiguration.h"
#include "SLAMModule.h"
#include "Image.h"

class SLAMEngine
{
public:

    SLAMEngine();

    ~SLAMEngine();

    bool initialize(
        StereoRigCalibrationDataPtr calibration,
        SLAMConfigurationPtr configuration);

    bool processFrame(int rank_in_recording, Image& image);

    bool finalize(SLAMReconstructionPtr& reconstruction);

protected:

    SLAMContextPtr mContext;
    std::vector<SLAMModulePtr> mModules;
};

typedef std::shared_ptr<SLAMEngine> SLAMEnginePtr;

