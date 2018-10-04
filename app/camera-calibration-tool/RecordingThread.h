
#pragma once

#include <QThread>
#include "Parameters.h"
#include "Output.h"
#include "Image.h"

class RecordingThread : public QThread
{
public:

    RecordingThread(Parameters* parameters, Output* output, QObject* parent=nullptr);

    ~RecordingThread();

protected:

    Parameters* mParameters;
    Output* mOutput;
    int mNumFrames;

protected:

    void run() override;
};

