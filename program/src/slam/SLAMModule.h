#pragma once

#include "SLAMProject.h"

class SLAMModule
{
public:

    SLAMModule(SLAMProjectPtr project);
    virtual ~SLAMModule();

    SLAMProjectPtr getProject();

    virtual void run(FrameList& frames) = 0;

private:

    SLAMProjectPtr mProject;
};
