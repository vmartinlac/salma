
#pragma once

#include <QThread>
#include "RecordingParameters.h"
#include "RecordingStatsWidget.h"
#include "VideoWidget.h"
#include "Image.h"

class RecordingThread : public QThread
{
public:

    RecordingThread(
        RecordingParameters* parameters,
        VideoInputPort* video,
        RecordingStatsInputPort* stats,
        QObject* parent=nullptr);

    ~RecordingThread();

protected:

    RecordingParameters* mParameters;
    RecordingStatsInputPort* mStats;
    VideoInputPort* mVideo;
    int mNumFrames;

protected:

    void run() override;
};

