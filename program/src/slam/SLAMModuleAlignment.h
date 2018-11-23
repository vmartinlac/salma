
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleAlignment : public SLAMModule
{
public:

    SLAMModuleAlignment(SLAMProjectPtr project);

    void run(FramePtr frame);
};

typedef std::shared_ptr<SLAMModuleAlignment> SLAMModuleAlignmentPtr;

