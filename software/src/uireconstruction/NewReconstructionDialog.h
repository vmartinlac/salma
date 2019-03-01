
#pragma once

#include <QLineEdit>
#include "NewOperationDialog.h"
#include "CalibrationListWidget.h"
#include "RecordingListWidget.h"

class NewReconstructionDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewReconstructionDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void accept() override;

protected:

    QWidget* createNameAndInputTab();
    QWidget* createConfigurationTab();

protected:

    QLineEdit* mName;
    RecordingListWidget* mRecording;
    CalibrationListWidget* mCalibration;
};

