#pragma once

#include <memory>
#include "VideoWidget.h"
#include "StatsWidget.h"

class Operation
{
public:

    Operation();

    virtual ~Operation() = 0;

    void setPorts( VideoInputPort* video, StatsInputPort* stats );

    virtual void before();
    virtual bool step() = 0;
    virtual void after();

protected:

    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;
};

typedef std::shared_ptr<Operation> OperationPtr;

