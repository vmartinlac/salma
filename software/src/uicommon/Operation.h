#pragma once

#include <memory>
#include "VideoWidget.h"
#include "StatsWidget.h"

class Project;
class QWidget;

class Operation
{
public:

    Operation();

    virtual ~Operation();

    void setPorts( VideoInputPort* video, StatsInputPort* stats );

    VideoInputPort* videoPort();
    StatsInputPort* statsPort();

    virtual const char* getName() = 0;

    virtual bool uibefore(QWidget* parent, Project* p);
    virtual bool before();
    virtual bool step() = 0;
    virtual void after();
    virtual void uiafter(QWidget* parent, Project* p);

private:

    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;
};

typedef std::shared_ptr<Operation> OperationPtr;

