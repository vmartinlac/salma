#pragma once

#include <QMainWindow>
#include <QAction>
#include "VideoWidget.h"
#include "InformationWidget.h"
#include "RecordingThread.h"
#include "Output.h"
#include "Parameters.h"

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
    InformationWidget* mInformationWidget;

    RecordingThread* mEngine;
    Parameters* mParameters;
    Output* mOutput;

protected slots:

    void configure();
    void startRecording();
    void stopRecording();
    void about();

    void engineStarted();
    void engineStopped();
};

