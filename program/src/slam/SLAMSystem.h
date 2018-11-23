#pragma once

#include <memory>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "VideoReader.h"
#include "SLAMDataStructures.h"
#include "SLAMProject.h"
#include "SLAMModuleFeatures.h"
#include "SLAMModuleStereoMatcher.h"
#include "SLAMModuleTemporalMatcher.h"
#include "SLAMModuleTriangulation.h"
#include "SLAMModuleAlignment.h"

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

    SLAMModuleFeaturesPtr mModuleFeatures;
    SLAMModuleStereoMatcherPtr mModuleStereoMatcher;
    SLAMModuleTemporalMatcherPtr mModuleTemporalMatcher;
    SLAMModuleTriangulationPtr mModuleTriangulation;
    SLAMModuleAlignmentPtr mModuleAlignment;

    FramePtr mFirstFrame;
    FramePtr mCurrentFrame;
};

typedef std::shared_ptr<SLAMSystem> SLAMSystemPtr;

