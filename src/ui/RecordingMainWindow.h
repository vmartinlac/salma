#pragma once

#include <QMainWindow>
#include <QAction>
#include "VideoWidget.h"
#include "RecordingStatsWidget.h"
#include "RecordingThread.h"
#include "RecordingParameters.h"

class RecordingMainWindow : public QMainWindow
{
    Q_OBJECT

public:

    RecordingMainWindow(QWidget* parent=nullptr);

protected:

    QAction* mActionConfigure;
    QAction* mActionStart;
    QAction* mActionStop;
    QAction* mActionAbout;

    RecordingParameters* mParameters;
    VideoWidget* mVideoWidget;
    RecordingStatsWidget* mStatsWidget;
    RecordingThread* mEngine;

protected slots:

    void configure();
    void startRecording();
    void stopRecording();
    void about();

    void engineStarted();
    void engineStopped();
};

