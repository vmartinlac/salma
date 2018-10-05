#pragma once

#include <QMainWindow>
#include <QAction>
#include "RecordingVideoWidget.h"
#include "RecordingInformationWidget.h"
#include "RecordingThread.h"
#include "RecordingOutput.h"
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

    RecordingVideoWidget* mVideoWidget;
    RecordingInformationWidget* mInformationWidget;

    RecordingThread* mEngine;
    RecordingParameters* mParameters;
    RecordingOutput* mOutput;

protected slots:

    void configure();
    void startRecording();
    void stopRecording();
    void about();

    void engineStarted();
    void engineStopped();
};

