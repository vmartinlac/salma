#include <QVBoxLayout>
#include <QSettings>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewCameraCalibrationDialog.h"
#include "VideoSystem.h"
#include "CameraCalibrationOperation.h"

NewCameraCalibrationDialog::NewCameraCalibrationDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mCamera = new CameraList();
    mTargetScale = new TargetScaleWidget();

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
    QString name;
    int camera_id = -1;
    VideoSourcePtr camera;
    double scale = 1.0;

    OperationPtr op;
    bool ok = true;
    const char* err = "";

    if(ok)
    {
        name = mName->text();
        ok = (name.isEmpty() == false);
        err = "Incorrect name!";
    }

    if(ok)
    {
        scale = mTargetScale->getScale(ok);
        err = "Incorrect target scale!";
    }

    if(ok)
    {
        camera_id = mCamera->getCameraId();
        ok = (camera_id >= 0);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        camera = VideoSystem::instance()->createVideoSourceGenICamMono(camera_id);
        ok = bool(camera);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        CameraCalibrationOperation* myop = new CameraCalibrationOperation();
        myop->mCalibrationName = mName->text().toStdString();
        myop->mCamera = camera;
        myop->mTargetCellLength = scale;
        op.reset(myop);
    }

    if(ok)
    {
        setOperation(op);
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

