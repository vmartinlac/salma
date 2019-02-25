#pragma once

#include <QLabel>
#include <QDialog>
#include <QSlider>
#include <QListWidget>
#include "Project.h"
#include "ManualCalibrationParameters.h"

class ManualCalibrationView;

class ManualCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    ManualCalibrationDialog(
        Project* project,
        ManualCalibrationParametersPtr params,
        QWidget* parent=nullptr);

protected:

    void accept() override;

protected slots:

    void setFrame(int);
    void setModeToLeft();
    void setModeToRight();
    void setModeToStereo();
    void setModeToPhotometric();

protected:

    Project* mProject;
    ManualCalibrationParametersPtr mParameters;

    QSlider* mSlider;
    QLabel* mLabelFrame;
    ManualCalibrationView* mView;
    QListWidget* mDataFrameList;
};

