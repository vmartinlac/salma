#pragma once

#include <QDialog>
#include <QAction>
#include "Operation.h"
#include "OperationThread.h"
#include "VideoWidget.h"
#include "StatsWidget.h"

class OperationDialog : public QDialog
{
    Q_OBJECT

public:

    OperationDialog(OperationPtr op, QWidget* parent=nullptr);
    ~OperationDialog();

protected:

    QAction* mActionConfigure;
    QAction* mActionQuit;
    QAction* mActionStart;
    QAction* mActionStop;
    QAction* mActionAbout;
    OperationPtr mOperation;
    VideoWidget* mVideoWidget;
    StatsWidget* mStatsWidget;
    OperationThread* mThread;
};

