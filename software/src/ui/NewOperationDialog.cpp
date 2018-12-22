#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "NewOperationDialog.h"

NewOperationDialog::NewOperationDialog(Project* project, QWidget* parent) : QDialog(parent)
{
    mTargetScale = new QLineEdit();
    mCamera = new CameraList();
    mName = new QLineEdit();
    mProject = project;

    QFormLayout* lay = new QFormLayout();
}

