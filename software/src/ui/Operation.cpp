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


