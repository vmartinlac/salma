#pragma once

#include <QLabel>
#include <QDialog>
#include <QSlider>
#include "Project.h"
#include "ManualCameraCalibrationParameters.h"

class ManualCameraCalibrationView;

class ManualCameraCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    ManualCameraCalibrationDialog(
        Project* project,
        ManualCameraCalibrationParametersPtr params,
        QWidget* parent=nullptr);

protected:

    void accept() override;

protected slots:

    void setFrame(int);

protected:

    Project* mProject;
    ManualCameraCalibrationParametersPtr mParameters;

    QSlider* mSlider;
    QLabel* mLabelFrame;
    ManualCameraCalibrationView* mView;
};

