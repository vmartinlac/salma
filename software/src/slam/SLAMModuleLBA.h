
#pragma once

#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleLBA : public SLAMModule
{
public:

    SLAMModuleLBA(SLAMContextPtr con);
    ~SLAMModuleLBA() override;

    bool init() override;
    void operator()() override;

protected:

    int mNumPreviousFrames;
    double mSigmaProjection;
    StereoRigCalibrationDataPtr mRig;
};

