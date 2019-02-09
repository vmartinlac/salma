#include <QVBoxLayout>
#include <QSettings>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewManualCameraCalibrationDialog.h"
#include "RecordingHeader.h"
#include "VideoSystem.h"
#include "CameraCalibrationOperation.h"
#include "Project.h"

NewManualCameraCalibrationDialog::NewManualCameraCalibrationDialog(Project* proj, QWidget* parent) : QDialog(parent)
{
    mProject = proj;

    mName = new QLineEdit();
    mRecording = new RecordingListWidget(mProject);
    mTargetScale = new TargetScaleWidget();

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Recording:", mRecording);
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
    setWindowTitle("New Camera Calibration");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void NewManualCameraCalibrationDialog::accept()
{
    ManualCameraCalibrationParametersPtr params(new ManualCameraCalibrationParameters());

    int recording_id = -1;

    bool ok = true;
    const char* err = "";

    if(ok)
    {
        params->name = mName->text();
        ok = (params->name.isEmpty() == false);
        err = "Incorrect name!";
    }

    if(ok)
    {
        params->scale = mTargetScale->getScale(ok);
        err = "Incorrect target scale!";
    }

    if(ok)
    {
        recording_id = mRecording->getRecordingId();
        ok = (recording_id >= 0);
        err = "Incorrect recording!";
    }

    if(ok)
    {
        mProject->loadRecording(recording_id, params->recording);
        ok = bool(params->recording);
        err = "Could not load recording!";
    }

    if(ok)
    {
        ok = (params->recording->num_views == 1);
        err = "You must select a mono recording!";
    }

    if(ok)
    {
        mParameters = std::move(params);
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

ManualCameraCalibrationParametersPtr NewManualCameraCalibrationDialog::getParameters()
{
    return mParameters;
}

