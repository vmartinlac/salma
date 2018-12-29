#pragma once

#include <QDialog>
#include <QAction>
#include "Operation.h"
#include "OperationThread.h"
#include "VideoWidget.h"
#include "StatsWidget.h"

class Project;

class OperationDialog : public QDialog
{
    Q_OBJECT

public:

    OperationDialog(Project* project, OperationPtr op, QWidget* parent=nullptr);
    ~OperationDialog();

protected slots:

    void startOperation();
    void stopOperation();
    void operationStarted();
    void operationStopped();

protected:

    QAction* mActionStart;
    QAction* mActionStop;
    Project* mProject;
    OperationPtr mOperation;
    VideoWidget* mVideoWidget;
    StatsWidget* mStatsWidget;
    OperationThread* mThread;
};

