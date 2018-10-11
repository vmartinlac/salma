
#pragma once

#include <QThread>
#include <QObject>
#include "Operation.h"
#include "StatsWidget.h"
#include "VideoWidget.h"
#include "Image.h"

class OperationThread : public QThread
{
public:

    OperationThread(
        VideoInputPort* video,
        StatsInputPort* stats,
        QObject* parent=nullptr);

    ~OperationThread();

    void setOperation(OperationPtr op);

protected:

    OperationPtr mOperation;
    VideoInputPort* mVideoPort;
    StatsInputPort* mStatsPort;

protected:

    void run() override;
};

