
#pragma once

#include <QThread>
#include "RecordingParameters.h"
#include "RecordingOutput.h"
#include "Image.h"

class RecordingThread : public QThread
{
public:

    RecordingThread(RecordingParameters* parameters, RecordingOutput* output, QObject* parent=nullptr);

    ~RecordingThread();

protected:

    RecordingParameters* mParameters;
    RecordingOutput* mOutput;
    int mNumFrames;

protected:

    void run() override;
};

