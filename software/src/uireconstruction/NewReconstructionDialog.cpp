#include <QTabWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QSettings>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewReconstructionDialog.h"
#include "VideoSystem.h"
#include "ReconstructionOperation.h"
#include "SLAMConfiguration.h"

NewReconstructionDialog::NewReconstructionDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    QTabWidget* tab = new QTabWidget();
    tab->addTab(createNameAndInputTab(), "Name and Input");
    tab->addTab(createConfigurationTab(), "Configuration");

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addWidget(tab);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New Reconstruction");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

QWidget* NewReconstructionDialog::createNameAndInputTab()
{
    mName = new QLineEdit();
    mRecording = new RecordingListWidget(project());
    mCalibration = new CalibrationListWidget(project());

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Recording:", mRecording);
    form->addRow("Rig calibration:", mCalibration);

    QWidget* ret = new QWidget;
    ret->setLayout(form);

    return ret;
}

QWidget* NewReconstructionDialog::createConfigurationTab()
{
    return new QLabel("TODO");
}

void NewReconstructionDialog::accept()
{
    QString name;
    int calibration_id = -1;
    int recording_id = -1;
    StereoRigCalibrationPtr calibration;
    RecordingHeaderPtr recording;

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
        calibration_id = mCalibration->getCalibrationId();
        ok = (calibration_id >= 0);
        err = "Incorrect rig calibration!";
    }

    if(ok)
    {
        recording_id = mRecording->getRecordingId();
        ok = (recording_id >= 0);
        err = "Incorrect recording!";
    }

    if(ok)
    {
        ok = project()->loadCalibration(calibration_id, calibration);
        err = "Could not load rig calibration!";
    }

    if(ok)
    {
        ok = project()->loadRecording(recording_id, recording);
        err = "Could not load recording!";
    }

    if(ok)
    {
        ok = (recording->num_views == 2);
        err = "Recording must be stereo!";
    }

    if(ok)
    {
        ReconstructionOperation* myop = new ReconstructionOperation();
        myop->mReconstructionName = mName->text().toStdString();
        myop->mCalibration = calibration;
        myop->mRecordingHeader = recording;
        myop->mConfiguration.reset(new SLAMConfiguration());
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

