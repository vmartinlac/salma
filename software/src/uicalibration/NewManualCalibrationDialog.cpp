#include <QVBoxLayout>
#include <QSettings>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewManualCalibrationDialog.h"
#include "ManualCalibrationParameters.h"
#include "RecordingHeader.h"
#include "VideoSystem.h"
#include "Project.h"

NewManualCalibrationDialog::NewManualCalibrationDialog(Project* proj, QWidget* parent) : QDialog(parent)
{
    mProject = proj;

    mName = new QLineEdit();
    mRecording = new RecordingListWidget(mProject);

    mName->setText("my calibration");

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Recording:", mRecording);

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

void NewManualCalibrationDialog::accept()
{
    ManualCalibrationParametersPtr params(new ManualCalibrationParameters());

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
        ok = (params->recording->num_views() == 2);
        err = "You must select a stereo recording!";
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

ManualCalibrationParametersPtr NewManualCalibrationDialog::getParameters()
{
    return mParameters;
}

