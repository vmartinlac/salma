
#pragma once

#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"
#include "MVPnPRANSACLM.h"

class SLAMModule1Alignment : public SLAMModule
{
public:

    SLAMModule1Alignment(SLAMContextPtr con);
    ~SLAMModule1Alignment() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    std::shared_ptr<MVPnP::SolverRANSACLM> mSolver;
};

