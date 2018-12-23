#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewCameraCalibrationDialog.h"

NewCameraCalibrationDialog::NewCameraCalibrationDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mCamera = new CameraList();
    mTargetScale = new QLineEdit();

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Camera:", mCamera);
    form->addRow("Target scale:", mTargetScale);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New camera calibration");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void NewCameraCalibrationDialog::accept()
{
    OperationPtr op;
    bool ok = true;

    // TODO

    if(ok)
    {
        setOperation(op);
        QDialog::accept();
    }
}

