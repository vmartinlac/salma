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

    void setProject( Project* proj );
    void setPorts( VideoInputPort* video, StatsInputPort* stats );

    Project* project();
    VideoInputPort* videoPort();
    StatsInputPort* statsPort();

    virtual bool before();
    virtual bool step() = 0;
    virtual void after();

    virtual const char* getName() = 0;

private:

    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;
    Project* mProject;
};

typedef std::shared_ptr<Operation> OperationPtr;

