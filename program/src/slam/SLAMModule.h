#pragma once

#include "SLAMProject.h"

class SLAMModule
{
public:

    SLAMModule(SLAMProjectPtr project);

    SLAMProjectPtr getProject();

    virtual void run(FrameList& frames) = 0;

private:

    SLAMProjectPtr mProject;
};
