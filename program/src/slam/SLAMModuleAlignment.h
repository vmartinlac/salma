
#pragma once

#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"
#include "MVPnP.h"

class SLAMModuleAlignment : public SLAMModule
{
public:

    SLAMModuleAlignment(SLAMProjectPtr project);

    void run(FramePtr frame);

protected:

    CameraCalibrationDataPtr mLeftCamera;
    CameraCalibrationDataPtr mRightCamera;
    StereoRigCalibrationDataPtr mRig;

    std::shared_ptr<MVPnP::Solver> mSolver;
};

typedef std::shared_ptr<SLAMModuleAlignment> SLAMModuleAlignmentPtr;

