#include "MVPnPImpl.h"
#include "MVPnPMono.h"
#include "MVPnP.h"

MVPnP::Solver::Solver()
{
}

MVPnP::Solver::~Solver()
{
}

MVPnP::Solver* MVPnP::Solver::create()
{
    return new SolverImpl();
    //return new SolverMono();
}

