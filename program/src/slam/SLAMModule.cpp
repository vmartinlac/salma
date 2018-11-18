#include "SLAMModule.h"

SLAMModule::SLAMModule(SLAMProjectPtr project)
{
    mProject = std::move(project);
}

SLAMProjectPtr SLAMModule::getProject()
{
    return mProject;
}

