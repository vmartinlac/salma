
#pragma once

#include <QThread>
#include <QDir>
#include "Camera.h"
#include "Image.h"

class RecordingThread : public QThread
{
public:

    RecordingThread()
    {
        mNumFrames = 0;
    }

    void setCamera(CameraPtr camera)
    {
        mCamera = camera;
    }

    void setOutputDirectoy(const QDir& dir)
    {
        mOutputDirectory = dir;
    }

protected:

    CameraPtr mCamera;
    QDir mOutputDirectory;
    int mNumFrames;

protected:

    void run() override;
};

