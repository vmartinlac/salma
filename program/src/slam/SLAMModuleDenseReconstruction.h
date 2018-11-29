
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleDenseReconstruction : public SLAMModule
{
public:

    SLAMModuleDenseReconstruction(SLAMProjectPtr project);

    void run(FramePtr frame);
};

typedef std::shared_ptr<SLAMModuleDenseReconstruction> SLAMModuleDenseReconstructionPtr;

