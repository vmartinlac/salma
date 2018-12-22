#pragma once

#include <QDialog>
#include <QLineEdit>
#include "PathWidget.h"
#include "CameraList.h"
#include "Project.h"

class NewOperationDialog : public QDialog
{
    Q_OBJECT

public:

    NewOperationDialog(Project* project, QWidget* parent=nullptr);

protected:

    Project* mProject;
    QLineEdit* mName;
    CameraList* mCamera;
    QLineEdit* mTargetScale;
};

