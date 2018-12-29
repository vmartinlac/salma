#pragma once

#include <memory>
#include "VideoWidget.h"
#include "StatsWidget.h"

class Project;

class Operation
{
public:

    Operation();

    virtual ~Operation();

    void setPorts( VideoInputPort* video, StatsInputPort* stats );

    VideoInputPort* videoPort();
    StatsInputPort* statsPort();

    virtual const char* getName() = 0;

    virtual bool before() = 0;
    virtual bool step() = 0;
    virtual void after() = 0;

    virtual bool success() = 0;
    virtual bool saveResult(Project* project) = 0;
    virtual void discardResult() = 0;

private:

    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;
};

typedef std::shared_ptr<Operation> OperationPtr;

