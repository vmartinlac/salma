#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewRigCalibrationDialog.h"

NewRigCalibrationDialog::NewRigCalibrationDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mLeftCamera = new CameraList();
    mRightCamera = new CameraList();
    mTargetScale = new QLineEdit();

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Left camera:", mLeftCamera);
    form->addRow("Right camera:", mRightCamera);
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
    setWindowTitle("New rig calibration");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void NewRigCalibrationDialog::accept()
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

