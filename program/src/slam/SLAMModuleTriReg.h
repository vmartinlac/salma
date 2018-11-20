
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriReg : public SLAMModule
{
public:

    SLAMModuleTriReg(SLAMProjectPtr project);

    void run(FramePtr frame);
};

typedef std::shared_ptr<SLAMModuleTriReg> SLAMModuleTriRegPtr;

