
#pragma once

#include <QLineEdit>
#include <QRadioButton>
#include <QCheckBox>
#include "NewOperationDialog.h"
#include "CameraList.h"
#include "FrameRateWidget.h"
#include "PathWidget.h"

class NewRecordingDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewRecordingDialog(int num_cameras, Project* proj, QWidget* parent=nullptr);
    ~NewRecordingDialog();

protected slots:

    void accept() override;

    void triggerChanged();

protected:

    QLineEdit* mName;
    std::vector<CameraList*> mCameras;
    FrameRateWidget* mFrameRate;
    QCheckBox* mVisualizationOnly;
    QRadioButton* mSoftwareTrigger;
    QRadioButton* mHardwareTrigger;
    PathWidget* mHardwareTriggerPath;
};

