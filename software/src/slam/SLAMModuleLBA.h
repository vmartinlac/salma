
#pragma once

#include <random>
#include "StereoRigCalibrationData.h"
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleLBA : public SLAMModule
{
public:

    SLAMModuleLBA(SLAMContextPtr con);
    ~SLAMModuleLBA() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    std::default_random_engine mEngine;
    StereoRigCalibrationDataPtr mRig;
};

