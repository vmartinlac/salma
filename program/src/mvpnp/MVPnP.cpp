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

MVPnP::Solver* MVPnP::Solver::create()
{
    return new SolverRANSACLM();
    //return new SolverLM();
    //return new SolverMonoOpenCV();
}

