#include "SLAMModule.h"

SLAMModule::SLAMModule(SLAMProjectPtr project)
{
    mProject = std::move(project);
}

SLAMModule::~SLAMModule()
{
}

SLAMProjectPtr SLAMModule::getProject()
{
    return mProject;
}

