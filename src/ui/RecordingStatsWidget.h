#pragma once

#include <QWidget>
#include <QLabel>
#include "Port.h"

struct RecordingStatsInputData
{
    std::string camera_name;
    std::string output_directory;
    int frame_count;
    int image_width;
    int image_height;
};

typedef Port<RecordingStatsInputData> RecordingStatsInputPort;

class RecordingStatsWidget : public QWidget
{
    Q_OBJECT

public:

    RecordingStatsWidget(QWidget* parent=nullptr);

    RecordingStatsInputPort* getPort();

protected slots:

    void refresh();

protected:

    QLabel* mLabelCamera;
    QLabel* mLabelOutputDirectory;
    QLabel* mLabelNumFrames;
    QLabel* mLabelResolution;
    RecordingStatsInputPort* mPort;
};

