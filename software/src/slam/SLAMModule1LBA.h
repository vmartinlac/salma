
#pragma once

#include <random>
#include <g2o/core/sparse_optimizer.h>
#include "StereoRigCalibration.h"
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1LBA : public SLAMModule
{
public:

    SLAMModule1LBA(SLAMContextPtr con);
    ~SLAMModule1LBA() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    std::default_random_engine mEngine;
    StereoRigCalibrationPtr mRig;
    g2o::SparseOptimizer mGraph;
};

