#include "Operation.h"

Operation::Operation()
{
}

Operation::~Operation()
{
}

void Operation::setPorts( VideoInputPort* video, StatsInputPort* stats )
{
    mVideoPort = video;
    mStatsPort = stats;
}

void Operation::setProject(Project* proj)
{
    mProject = proj;
}

bool Operation::before()
{
    return true;
}

void Operation::after()
{
}

Project* Operation::project()
{
    return mProject;
}

VideoInputPort* Operation::videoPort()
{
    return mVideoPort;
}

StatsInputPort* Operation::statsPort()
{
    return mStatsPort;
}

