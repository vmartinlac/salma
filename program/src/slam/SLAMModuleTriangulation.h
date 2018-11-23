
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriangulation : public SLAMModule
{
public:

    SLAMModuleTriangulation(SLAMProjectPtr project);

    void run(FramePtr frame);

protected:

    bool check(FramePtr frame, int left_kpt0, int right_kpt0);
    void connect(FramePtr frame, int left_kpt0, int right_kpt0);

protected:

    int mMaxNumberOfPreviousFrames;
};

typedef std::shared_ptr<SLAMModuleTriangulation> SLAMModuleTriangulationPtr;

