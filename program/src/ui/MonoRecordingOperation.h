
#pragma once

#include <QTime>
#include <QDir>
#include <fstream>
#include <iostream>
#include "VideoSource.h"
#include "Operation.h"

class MonoRecordingOperation : public Operation
{
public:

    MonoRecordingOperation();

    ~MonoRecordingOperation() override;

    bool before() override;
    bool step() override;
    void after() override;

public:

    QDir mOutputDirectory;
    VideoSourcePtr mCamera;

protected:

    QTime mClock;
    int mNumFrames;
    std::ofstream mOutputCSV;
};

