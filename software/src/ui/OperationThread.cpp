#include "OperationThread.h"

OperationThread::OperationThread(
    VideoInputPort* video,
    StatsInputPort* stats,
    QObject* parent) : QThread(parent)
{
    mVideoPort = video;
    mStatsPort = stats;
}

OperationThread::~OperationThread()
{
}

void OperationThread::setOperation(OperationPtr op)
{
    mOperation = std::move(op);
}

void OperationThread::run()
{
    if(mOperation)
    {
        mOperation->setPorts(mVideoPort, mStatsPort);

        bool go_on = mOperation->before();

        while( go_on && isInterruptionRequested() == false )
        {
            go_on = mOperation->step();
        }

        mOperation->after();
    }
}

