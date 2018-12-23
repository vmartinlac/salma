#pragma once

#include "NewOperationDialog.h"
#include "CameraList.h"
#include <QLineEdit>

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
    // left calibration
    // right calibration
    QLineEdit* mTargetScale;
};

