
#pragma once

#include <QTime>
#include <QDir>
#include <fstream>
#include <iostream>
#include "VideoSource.h"
#include "Operation.h"

class Project;

class RecordingOperation : public Operation
{
public:

    RecordingOperation();

    ~RecordingOperation() override;

    const char* getName() override;

    bool before() override;
    bool step() override;
    void after() override;

    bool success() override;
    bool saveResult(Project* p) override;
    void discardResult() override;

public:

    std::string mRecordingName;
    VideoSourcePtr mCamera;
    bool mVisualizationOnly;
    double mMaxFrameRate;
    QString mDirectory;
    QDir mOutputDirectory;

protected:

    QTime mClock;
    int mNumFrames;
    std::ofstream mOutputCSV;
    QTime mFrameRateClock;
};

