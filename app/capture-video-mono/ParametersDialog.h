#pragma once

#include <QDialog>
#include <QDir>
#include "Camera.h"

class ParametersDialog : public QDialog
{
public:

    ParametersDialog(QWidget* parent=nullptr);

protected:

    QDir mOutputDirectory;
    CameraPtr mCamera;
};

