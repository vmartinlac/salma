#include "MVPnP.h"
#include "MVPnPRANSACLM.h"
#include "MVPnPLM.h"
#include "MVPnPMonoOpenCV.h"

MVPnP::Solver::Solver()
{
}

MVPnP::Solver::~Solver()
{
}

double MVPnP::Solver::computeReprojectionError(
    const std::vector<View>& views,
    const Sophus::SE3d& rig_to_world)
{
    return 0.0;
}

double MVPnP::Solver::computeReprojectionError(
    const std::vector<View>& views,
    const std::vector< std::vector<bool> >& inliers,
    const Sophus::SE3d& rig_to_world)
{
    return 0.0;
}
