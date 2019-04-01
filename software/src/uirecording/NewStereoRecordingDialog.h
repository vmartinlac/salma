
#pragma once

#include <QLineEdit>
#include <QCheckBox>
#include "NewOperationDialog.h"
#include "CameraList.h"
#include "FrameRateWidget.h"

class NewStereoRecordingDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewStereoRecordingDialog(Project* proj, QWidget* parent=nullptr);
    ~NewStereoRecordingDialog();

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
    CameraList* mLeftCamera;
    CameraList* mRightCamera;
    FrameRateWidget* mFrameRate;
    QCheckBox* mVisualizationOnly;
    QCheckBox* mSoftwareTrigger;
};

