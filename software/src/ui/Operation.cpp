#include "Operation.h"

Operation::Operation()
{
	mVideoPort = nullptr;
	mStatsPort = nullptr;
}

Operation::~Operation()
{
}

void Operation::setPorts( VideoInputPort* video, StatsInputPort* stats )
{
    mVideoPort = video;
    mStatsPort = stats;
}

VideoInputPort* Operation::videoPort()
{
    return mVideoPort;
}

StatsInputPort* Operation::statsPort()
{
    return mStatsPort;
}

bool Operation::uibefore(QWidget* parent, Project* p)
{
    return true;
}

bool Operation::before()
{
    return true;
}

void Operation::after()
{
}

void Operation::uiafter(QWidget* parent, Project* p)
{
}

