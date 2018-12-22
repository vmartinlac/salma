#pragma once

#include <QDialog>
#include <QLineEdit>
#include "PathWidget.h"
#include "CameraList.h"
#include "Project.h"

class NewCameraCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    NewCameraCalibrationDialog(Project* project, QWidget* parent=nullptr);

protected:

    Project* mProject;
    QLineEdit* mName;
    CameraList* mCamera;
    QLineEdit* mTargetScale;
};

