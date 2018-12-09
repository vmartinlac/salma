#pragma once

#include <memory>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "VideoReader.h"
#include "SLAMDataStructures.h"
#include "SLAMProject.h"

#include "SLAMModuleOpticalFlow.h"
#include "SLAMModuleAlignment.h"

#include "SLAMModuleFeatures.h"
#include "SLAMModuleStereoMatcher.h"
#include "SLAMModuleTriangulation.h"

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
    static void assignMapPointIds(FramePtr last_frame);
    static void freeOldImages(FramePtr frame, int num_to_keep);

protected:

    int mOptionFirst;
    int mOptionCount;
    std::string mOptionName;

    SLAMProjectPtr mProject;

    SLAMModuleOpticalFlowPtr mModuleOpticalFlow;
    SLAMModuleAlignmentPtr mModuleAlignment;

    SLAMModuleFeaturesPtr mModuleFeatures;
    SLAMModuleStereoMatcherPtr mModuleStereoMatcher;
    SLAMModuleTriangulationPtr mModuleTriangulation;

    SLAMModuleDenseReconstructionPtr mModuleDenseReconstruction;

    FramePtr mCurrentFrame;
};

typedef std::shared_ptr<SLAMSystem> SLAMSystemPtr;

