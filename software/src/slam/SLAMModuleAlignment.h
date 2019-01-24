
#pragma once

#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"
#include "MVPnPRANSACLM.h"

class SLAMModuleAlignment : public SLAMModule
{
public:

    SLAMModuleAlignment(SLAMContextPtr con);
    ~SLAMModuleAlignment() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    std::shared_ptr<MVPnP::SolverRANSACLM> mSolver;
};

