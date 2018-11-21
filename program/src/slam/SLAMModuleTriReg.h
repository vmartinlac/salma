
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriReg : public SLAMModule
{
public:

    SLAMModuleTriReg(SLAMProjectPtr project);

    void run(FramePtr frame);

protected:

    std::vector<MapPointPtr> mMapPoints;
};

typedef std::shared_ptr<SLAMModuleTriReg> SLAMModuleTriRegPtr;

