#pragma once

#include <QLabel>
#include <QDialog>
#include <QSlider>
#include "Project.h"
#include "ManualCameraCalibrationParameters.h"

class ManualCameraCalibrationDialog : public QDialog
{
public:

    ManualCameraCalibrationDialog(
        Project* project,
        ManualCameraCalibrationParametersPtr params,
        QWidget* parent=nullptr);

protected:

    Project* mProject;
    ManualCameraCalibrationParametersPtr mParameters;

    QSlider* mSlider;
    QLabel* mLabelFrame;
};

