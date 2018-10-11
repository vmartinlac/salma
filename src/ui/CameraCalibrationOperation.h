
#pragma once

#include <QDir>
#include "Camera.h"
#include "Operation.h"

class CameraCalibrationOperation : public Operation
{
public:

    CameraCalibrationOperation();

    ~CameraCalibrationOperation() override;

    OperationName getOperation() override;

    CameraCalibrationOperation* cameraCalibration() override;

    void before() override;
    bool step() override;
    void after() override;

public:

    std::string mOutputPath;
    CameraPtr mCamera;
};

