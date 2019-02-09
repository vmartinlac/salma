#pragma once

#include <QDialog>
#include <QLineEdit>
#include "RecordingListWidget.h"
#include "TargetScaleWidget.h"
#include "ManualCameraCalibrationParameters.h"

class NewManualCameraCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    NewManualCameraCalibrationDialog(Project* proj, QWidget* parent=nullptr);

    ManualCameraCalibrationParametersPtr getParameters();

protected slots:

    void accept() override;

protected:

    Project* mProject;

    QLineEdit* mName;
    RecordingListWidget* mRecording;
    TargetScaleWidget* mTargetScale;

    ManualCameraCalibrationParametersPtr mParameters;
};

