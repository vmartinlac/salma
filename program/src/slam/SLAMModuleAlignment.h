
#pragma once

#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"
#include "MVPnPRANSACLM.h"

class SLAMModuleAlignment : public SLAMModule
{
public:

    SLAMModuleAlignment(SLAMProjectPtr project);
    ~SLAMModuleAlignment() override;

    void run(FrameList& frames) override;

protected:

    CameraCalibrationDataPtr mLeftCamera;
    CameraCalibrationDataPtr mRightCamera;
    StereoRigCalibrationDataPtr mRig;

    std::shared_ptr<MVPnP::SolverRANSACLM> mSolver;
};

typedef std::shared_ptr<SLAMModuleAlignment> SLAMModuleAlignmentPtr;

