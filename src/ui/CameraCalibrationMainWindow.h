#pragma once

#include <QMainWindow>
#include <QAction>
#include "VideoWidget.h"
#include "CameraCalibrationStatsWidget.h"
#include "CameraCalibrationThread.h"
#include "CameraCalibrationParameters.h"

class CameraCalibrationMainWindow : public QMainWindow
{
    Q_OBJECT

public:

    CameraCalibrationMainWindow(QWidget* parent=nullptr);

protected:

    QAction* mActionConfigure;
    QAction* mActionStart;
    QAction* mActionStop;
    QAction* mActionAbout;

    CameraCalibrationParameters* mParameters;
    VideoWidget* mVideoWidget;
    CameraCalibrationStatsWidget* mStatsWidget;
    CameraCalibrationThread* mEngine;

protected slots:

    void configure();
    void startCameraCalibration();
    void stopCameraCalibration();
    void about();

    void engineStarted();
    void engineStopped();
};

