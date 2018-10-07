
#pragma once

#include <QThread>
#include "CameraCalibrationParameters.h"
#include "CameraCalibrationStatsWidget.h"
#include "VideoWidget.h"
#include "Image.h"

class CameraCalibrationThread : public QThread
{
public:

    CameraCalibrationThread(
        CameraCalibrationParameters* parameters,
        VideoInputPort* video,
        CameraCalibrationStatsInputPort* stats,
        QObject* parent=nullptr);

    ~CameraCalibrationThread();

protected:

    CameraCalibrationParameters* mParameters;
    CameraCalibrationStatsInputPort* mStats;
    VideoInputPort* mVideo;
    int mNumFrames;

protected:

    void run() override;
};

