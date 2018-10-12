#include "Operation.h"

Operation::Operation()
{
}

Operation::~Operation()
{
}

void Operation::setPorts(
    VideoInputPort* video,
    StatsInputPort* stats)
{
    mVideoPort = video;
    mStatsPort = stats;
}

bool Operation::before()
{
    return true;
}

void Operation::after()
{
}

