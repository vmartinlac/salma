#include <QFormLayout>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "NewStereoRecordingDialog.h"
#include "VideoSystem.h"
#include "RecordingOperation.h"

NewStereoRecordingDialog::NewStereoRecordingDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mLeftCamera = new CameraList();
    mRightCamera = new CameraList();
    mFrameRate = new FrameRateWidget();
    mVisualizationOnly = new QCheckBox();
    mSoftwareTrigger = new QCheckBox();

    mSoftwareTrigger->setChecked(true);

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Left camera:", mLeftCamera);
    form->addRow("Right camera:", mRightCamera);
    form->addRow("Max frame rate:", mFrameRate);
    form->addRow("Visualization only:", mVisualizationOnly);
    form->addRow("Software trigger:", mSoftwareTrigger);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New Stereo Recording");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

NewStereoRecordingDialog::~NewStereoRecordingDialog()
{
}

void NewStereoRecordingDialog::accept()
{
    QString name;
    int left_camera_id = -1;
    int right_camera_id = -1;
    GenICamVideoSourcePtr camera;
    double framerate;
    bool visualization_only;

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
        framerate = mFrameRate->getFrameRate();
    }

    if(ok)
    {
        left_camera_id = mLeftCamera->getCameraId();
        right_camera_id = mRightCamera->getCameraId();
        ok = (left_camera_id >= 0 && right_camera_id >= 0 && left_camera_id != right_camera_id);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        camera = VideoSystem::instance()->createGenICamVideoSourceStereo(left_camera_id, right_camera_id);
        ok = bool(camera);
        err = "Incorrect camera!";
    }

    if(ok)
    {
        if( mSoftwareTrigger->isChecked() )
        {
            camera->setSoftwareTrigger();
        }
        else
        {
            throw std::runtime_error("not implemented!");
        }
        err = "Incorrect trigger!";
    }

    if(ok)
    {
        visualization_only = mVisualizationOnly->isChecked();
    }

    if(ok)
    {
        RecordingOperation* myop = new RecordingOperation();
        myop->mRecordingName = name.toStdString();
        myop->mCamera = camera;
        myop->mVisualizationOnly = visualization_only;
        myop->mMaxFrameRate = framerate;
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

