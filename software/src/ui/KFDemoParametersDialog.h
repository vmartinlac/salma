#pragma once

#include <QDialog>
#include "CameraList.h"
#include "KFDEngine.h"
#include "TargetScaleWidget.h"
#include "CameraCalibrationListWidget.h"

class Project;

class KFDemoParametersDialog : public QDialog
{
public:

    KFDemoParametersDialog(Project* proj, KFDEngine* engine, QWidget* parent=nullptr);

public:

    virtual void accept();

protected:

    KFDEngine* mEngine;
    Project* mProject;
    CameraList* mCamera;
    CameraCalibrationListWidget* mCalibration;
    TargetScaleWidget* mTargetScale;
};

