
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleLocalBundleAdjustment : public SLAMModule
{
public:

    SLAMModuleLocalBundleAdjustment(SLAMContextPtr con);
    ~SLAMModuleLocalBundleAdjustment() override;

    bool init() override;
    void operator()() override;
};

