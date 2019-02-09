
#pragma once

#include <QLineEdit>
#include <QCheckBox>
#include "NewOperationDialog.h"
#include "CameraList.h"
#include "FrameRateWidget.h"

class NewMonoRecordingDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewMonoRecordingDialog(Project* proj, QWidget* parent=nullptr);
    ~NewMonoRecordingDialog();

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
    CameraList* mCamera;
    FrameRateWidget* mFrameRate;
    QCheckBox* mVisualizationOnly;
};

