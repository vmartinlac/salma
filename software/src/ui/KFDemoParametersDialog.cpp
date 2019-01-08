#include <QMessageBox>
#include <QFormLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include "KFDemoParametersDialog.h"
#include "VideoSystem.h"
#include "Project.h"

KFDemoParametersDialog::KFDemoParametersDialog(Project* proj, KFDEngine* engine, QWidget* parent) : QDialog(parent)
{
    mProject = proj;
    mEngine = engine;

    mCamera = new CameraList();
    mCalibration = new CameraCalibrationListWidget(proj);
    mTargetScale = new TargetScaleWidget();

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera:", mCamera);
    form->addRow("Calibration:", mCalibration);
    form->addRow("Target scale:", mTargetScale);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addLayout(form);
    lay->addLayout(hlay);

    setLayout(lay);
    setWindowTitle("Start demo");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void KFDemoParametersDialog::accept()
{
    int camera_id = -1;
    int calibration_id = -1;
    CameraCalibrationDataPtr calibration;
    VideoSourcePtr video;
    double target_scale = 1.0;

    bool ok = true;
    const char* err = "";

    if(ok)
    {
        target_scale = mTargetScale->getScale(ok);
        err = "Incorrect target scale!";
    }

    if(ok)
    {
        calibration_id = mCalibration->getCameraCalibrationId();
        ok = (calibration_id >= 0);
        err = "Please select a camera calibration!";
    }

    if(ok)
    {
        ok = mProject->loadCamera(calibration_id, calibration);
        err = "Could not load camera calibration!";
    }

    if(ok)
    {
        camera_id = mCamera->getCameraId();
        ok = (camera_id >= 0);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        video = VideoSystem::instance()->createVideoSourceGenICamMono(camera_id);
        ok = bool(video);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        mEngine->mCamera = video;
        mEngine->mCalibration = calibration;
        mEngine->mTargetScale = target_scale;
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

