#pragma once

#include "SLAMProject.h"

class SLAMModule
{
public:

    SLAMModule(SLAMProjectPtr project);

    SLAMProjectPtr getProject();

private:

    SLAMProjectPtr mProject;
};
