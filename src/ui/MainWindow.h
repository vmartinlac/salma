
#pragma once

#include <QMainWindow>
#include <QAction>
#include "VideoWidget.h"
#include "StatsWidget.h"
#include "OperationThread.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected:

    QAction* mActionConfigure;
    QAction* mActionStart;
    QAction* mActionStop;
    QAction* mActionAbout;

    VideoWidget* mVideoWidget;
    StatsWidget* mStatsWidget;

    OperationThread* mThread;

protected slots:

    void configure();
    void startOperation();
    void stopOperation();
    void about();

    void operationStarted();
    void operationStopped();
};

