
#pragma once

#include <QLineEdit>
#include "NewOperationDialog.h"
#include "RigCalibrationListWidget.h"
#include "RecordingListWidget.h"

class NewReconstructionDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewReconstructionDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
    RecordingListWidget* mRecording;
    RigCalibrationListWidget* mCalibration;
};

