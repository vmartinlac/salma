#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "NewCameraCalibrationDialog.h"

NewCameraCalibrationDialog::NewCameraCalibrationDialog(Project* project, QWidget* parent) : QDialog(parent)
{
    mTargetScale = new QLineEdit();
    mCamera = new CameraList();
    mName = new QLineEdit();
    mProject = project;

    QFormLayout* lay = new QFormLayout();

    setLayout(vlay);
}

