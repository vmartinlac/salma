#include "SLAMModuleTriReg.h"

SLAMModuleTriReg::SLAMModuleTriReg(SLAMProjectPtr project) : SLAMModule(project)
{
    const double scale_factor = project->getParameterReal("features_scale_factor", 1.1);
}

