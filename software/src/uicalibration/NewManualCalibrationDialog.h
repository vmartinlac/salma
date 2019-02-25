#pragma once

#include <QDialog>
#include <QLineEdit>
#include "RecordingListWidget.h"
#include "ManualCalibrationParameters.h"

class NewManualCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    NewManualCalibrationDialog(Project* proj, QWidget* parent=nullptr);

    ManualCalibrationParametersPtr getParameters();

protected slots:

    void accept() override;

protected:

    Project* mProject;

    QLineEdit* mName;
    RecordingListWidget* mRecording;

    ManualCalibrationParametersPtr mParameters;
};

