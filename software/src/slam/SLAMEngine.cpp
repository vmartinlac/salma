#include "SLAMModuleOpticalFlow.h"
#include "SLAMModuleAlignment.h"
#include "SLAMModuleFeatures.h"
#include "SLAMModuleStereoMatcher.h"
#include "SLAMModuleTriangulation.h"
#include "SLAMModuleDenseReconstruction.h"
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
}

SLAMEngine::~SLAMEngine()
{
}

bool SLAMEngine::initialize(
    StereoRigCalibrationDataPtr calibration,
    SLAMConfigurationPtr configuration)
{
    bool ok = true;

    mContext.reset(new SLAMContext);

    return ok;
}

bool SLAMEngine::processFrame(Image& image)
{
    return false;
}

bool SLAMEngine::finalize(SLAMReconstructionPtr& reconstruction)
{
    return false;
}

