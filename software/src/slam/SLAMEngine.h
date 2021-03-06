#pragma once

#include "SLAMDataStructures.h"
#include "StereoRigCalibration.h"
#include "SLAMConfiguration.h"
#include "SLAMModule.h"
#include "Image.h"

class SLAMEngine
{
public:

    SLAMEngine();

    ~SLAMEngine();

    bool initialize(
        StereoRigCalibrationPtr calibration,
        SLAMConfigurationPtr configuration);

    bool processFrame(int rank_in_recording, Image& image);

    bool finalize(SLAMReconstructionPtr& reconstruction);

protected:

    SLAMModuleId mNextModule;
    SLAMContextPtr mContext;
    std::vector<SLAMModulePtr> mModules;
};

typedef std::shared_ptr<SLAMEngine> SLAMEnginePtr;

