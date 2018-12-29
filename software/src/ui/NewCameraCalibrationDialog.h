#pragma once

#include "NewOperationDialog.h"
#include "CameraList.h"
#include "TargetScaleWidget.h"
#include <QLineEdit>

class NewCameraCalibrationDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewCameraCalibrationDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
    CameraList* mCamera;
    TargetScaleWidget* mTargetScale;
};

