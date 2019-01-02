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
    SLAMModulePtr mModuleOpticalFlow;
    SLAMModulePtr mModuleAlignment;
    SLAMModulePtr mModuleFeatures;
    SLAMModulePtr mModuleStereoMatcher;
    SLAMModulePtr mModuleTriangulation;
    SLAMModulePtr mModuleDenseReconstruction;
};

typedef std::shared_ptr<SLAMEngine> SLAMEnginePtr;

