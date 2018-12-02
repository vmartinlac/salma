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
#include "SLAMModuleDenseReconstruction.h"

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
    SLAMModuleDenseReconstructionPtr mModuleDenseReconstruction;

    FramePtr mFirstFrame;
    FramePtr mCurrentFrame;

    int mSkipTo;
};

typedef std::shared_ptr<SLAMSystem> SLAMSystemPtr;

