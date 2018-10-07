
#pragma once

#include <QWidget>
#include <QLabel>
#include "Port.h"

struct CameraCalibrationStatsInputData
{
    std::string camera_name;
    std::string output_path;
    int frame_count;
    int successful_tracking_count;
    int attempted_tracking_count;
};

typedef Port<CameraCalibrationStatsInputData> CameraCalibrationStatsInputPort;

class CameraCalibrationStatsWidget : public QWidget
{
    Q_OBJECT

public:

    CameraCalibrationStatsWidget(QWidget* parent=nullptr);

    CameraCalibrationStatsInputPort* getPort();

protected slots:

    void refresh();

protected:

    CameraCalibrationStatsInputPort* mPort;

    QLabel* mLabelCameraName;
    QLabel* mLabelOutputPath;
    QLabel* mLabelFrameCount;
    QLabel* mLabelTrackingCount;
};

