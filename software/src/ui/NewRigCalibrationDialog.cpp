#include <QVBoxLayout>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "VideoSystem.h"
#include "CameraCalibrationListWidget.h"
#include "NewRigCalibrationDialog.h"
#include "RigCalibrationOperation.h"

NewRigCalibrationDialog::NewRigCalibrationDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mLeftCamera = new CameraList();
    mRightCamera = new CameraList();
    mLeftCalibration = new CameraCalibrationListWidget(proj);
    mRightCalibration = new CameraCalibrationListWidget(proj);
    mTargetScale = new TargetScaleWidget();

    mProject = proj;

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Left camera:", mLeftCamera);
    form->addRow("Left camera calibration:", mLeftCalibration);
    form->addRow("Right camera:", mRightCamera);
    form->addRow("Right camera calibration:", mRightCalibration);
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
    QString name;
    int left_camera_id = -1;
    int right_camera_id = -1;
    int left_calibration_id = -1;
    int right_calibration_id = -1;
    CameraCalibrationDataPtr left_calibration;
    CameraCalibrationDataPtr right_calibration;
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
        left_calibration_id = mLeftCalibration->getCameraCalibrationId();
        right_calibration_id = mRightCalibration->getCameraCalibrationId();
        ok = ( left_calibration_id >= 0 && right_calibration_id >= 0 );
        err = "Incorrect calibration data!";
    }

    if(ok)
    {
        ok = mProject->loadCamera(left_calibration_id, left_calibration);
        err = "Could not load calibration data!";
    }

    if(ok)
    {
        ok = mProject->loadCamera(right_calibration_id, right_calibration);
        err = "Could not load calibration data!";
    }

    if(ok)
    {
        left_camera_id = mLeftCamera->getCameraId();
        right_camera_id = mRightCamera->getCameraId();
        ok = ( left_camera_id >= 0 && right_camera_id >= 0 && left_camera_id != right_camera_id );
        err = "Incorrect camera!";
    }

    if(ok)
    {
        camera = VideoSystem::instance()->createVideoSourceGenICamStereo(left_camera_id, right_camera_id);
        ok = bool(camera);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        RigCalibrationOperation* myop = new RigCalibrationOperation();
        myop->mCalibrationName = mName->text().toStdString();
        myop->mCamera = camera;
        myop->mTargetCellLength = scale;
        myop->mLeftCalibrationData = left_calibration;
        myop->mRightCalibrationData = right_calibration;
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

