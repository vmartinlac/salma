#pragma once

#include <memory>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "VideoReader.h"
#include "SLAMDataStructures.h"
#include "SLAMProject.h"
#include "SLAMModulePyramid.h"
#include "SLAMModuleFeatures.h"
#include "SLAMModuleStereoMatcher.h"

class SLAMSystem
{
public:

    SLAMSystem();
    ~SLAMSystem();

    void run();

protected:

    bool initialize();
    void finalize();
    void printWelcomeMessage();
    void handleFrame(FramePtr frame);

protected:

    SLAMProjectPtr mProject;

    SLAMModulePyramidPtr mModulePyramid;
    SLAMModuleFeaturesPtr mModuleFeatures;
    SLAMModuleStereoMatcherPtr mModuleStereoMatcher;

    FramePtr mFirstFrame;
    FramePtr mCurrentFrame;
};

typedef std::shared_ptr<SLAMSystem> SLAMSystemPtr;

