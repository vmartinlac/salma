#include "Operation.h"

Operation::Operation()
{
}

void Operation::setPorts(
    VideoInputPort* video,
    StatsInputPort* stats)
{
    mVideoPort = video;
    mStatsPort = stats;
}

void Operation::before()
{
}

void Operation::after()
{
}

