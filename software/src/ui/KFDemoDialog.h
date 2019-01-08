#pragma once

#include <QDialog>
#include <QAction>
#include "VideoWidget.h"
#include "KFDViewer.h"
#include "KFDEngine.h"

class Project;

class KFDemoDialog : public QDialog
{
    Q_OBJECT

public:

    KFDemoDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void onStartDemo();
    void onStopDemo();
    void onEngineStarted();
    void onEngineStopped();

protected:

    VideoWidget* mVideo;
    KFDViewer* mViewer;
    KFDEngine* mEngine;
    QAction* mActionStart;
    QAction* mActionStop;
    Project* mProject;
};

