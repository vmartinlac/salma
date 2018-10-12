
#pragma once

#include <QDir>
#include "Camera.h"
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

    QDir output_directory;
    CameraPtr mCamera;
};

