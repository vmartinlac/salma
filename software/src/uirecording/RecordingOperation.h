
#pragma once

#include <QTime>
#include <QDir>
#include <fstream>
#include <iostream>
#include <opencv2/videoio.hpp>
#include "VideoSource.h"
#include "Operation.h"
#include "RecordingHeader.h"

class Project;

class RecordingOperation : public Operation
{
public:

    RecordingOperation();

    ~RecordingOperation() override;

    const char* getName() override;

    bool uibefore(QWidget* parent, Project* project) override;
    bool before() override;
    bool step() override;
    void after() override;
    void uiafter(QWidget* parent, Project* project) override;

public:

    std::string mRecordingName;
    VideoSourcePtr mCamera;
    bool mVisualizationOnly;
    double mMaxFrameRate;

protected:

    bool mSuccess;
    QTime mClock;
    int mNumFrames;
    QTime mFrameRateClock;
    RecordingHeaderPtr mResult;
};

