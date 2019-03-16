
#pragma once

#include <QCheckBox>
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

    QCheckBox* mCheckDenseReconstruction;
    QCheckBox* mCheckDebugR;
    QCheckBox* mCheckDebugF;
    QCheckBox* mCheckDebugTM;
    QCheckBox* mCheckDebugA;
    QCheckBox* mCheckDebugKFS;
    QCheckBox* mCheckDebugLBA;
    QCheckBox* mCheckDebugSM;
    QCheckBox* mCheckDebugT;
    QCheckBox* mCheckDebugDR;

    RecordingListWidget* mRecording;
    CalibrationListWidget* mCalibration;
};

