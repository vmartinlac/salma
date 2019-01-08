
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleLBA : public SLAMModule
{
public:

    SLAMModuleLBA(SLAMContextPtr con);
    ~SLAMModuleLBA() override;

    bool init() override;
    void operator()() override;
};

