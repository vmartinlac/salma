#include "MVPnP.h"
#include "MVPnPImpl.h"


MVPnP::Solver* MVPnP::Solver::create()
{
    return new SolverImpl();
}

MVPnP::Solver::Solver()
{
}

MVPnP::Solver::~Solver()
{
}

