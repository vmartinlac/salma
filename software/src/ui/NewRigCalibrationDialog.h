#pragma once

#include "NewOperationDialog.h"
#include "CameraList.h"
#include "TargetScaleWidget.h"
#include <QLineEdit>

class CameraCalibrationListWidget;

class NewRigCalibrationDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewRigCalibrationDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
    CameraList* mLeftCamera;
    CameraList* mRightCamera;
    CameraCalibrationListWidget* mLeftCalibration;
    CameraCalibrationListWidget* mRightCalibration;
    TargetScaleWidget* mTargetScale;
};

