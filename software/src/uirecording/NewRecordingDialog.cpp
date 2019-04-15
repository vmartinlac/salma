#include <set>
#include <QFormLayout>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "NewRecordingDialog.h"
#include "VideoSystem.h"
#include "RecordingOperation.h"

NewRecordingDialog::NewRecordingDialog(int num_cameras, Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mName->setText("my recording");

    for(int i=0; i<num_cameras; i++)
    {
        mCameras.push_back( new CameraList() );
    }
    mFrameRate = new FrameRateWidget();
    mVisualizationOnly = new QCheckBox();
    mSoftwareTrigger = new QRadioButton();
    mHardwareTrigger = new QRadioButton();
    mHardwareTriggerPath = new PathWidget(PathWidget::GET_OPEN_FILENAME);

    mSoftwareTrigger->setChecked(true);
    mHardwareTriggerPath->setPath("/dev/ttyACM0");

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    for(int i=0; i<num_cameras; i++)
    {
        const QString text = "Camera #" + QString::number(i) + ":";
        form->addRow(text, mCameras[i]);
    }
    form->addRow("Max frame rate:", mFrameRate);
    form->addRow("Visualization only:", mVisualizationOnly);
    form->addRow("Software trigger:", mSoftwareTrigger);
    form->addRow("Hardware trigger:", mHardwareTrigger);
    form->addRow("Hardware trigger device:", mHardwareTriggerPath);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New Recording");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(mHardwareTrigger, SIGNAL(clicked()), this, SLOT(triggerChanged()));
    connect(mSoftwareTrigger, SIGNAL(clicked()), this, SLOT(triggerChanged()));

    triggerChanged();
}

NewRecordingDialog::~NewRecordingDialog()
{
}

void NewRecordingDialog::accept()
{
    QString name;
    std::vector<int> camera_ids(mCameras.size(), -1);
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
        for(size_t i=0; ok && i<mCameras.size(); i++)
        {
            camera_ids[i] = mCameras[i]->getCameraId();
            ok = (camera_ids[i] >= 0);
        }

        if(ok)
        {
            std::set<int> items(camera_ids.begin(), camera_ids.end());
            ok = ( items.size() == mCameras.size() );
        }

        err = "Incorrect camera!";
    }

    if(ok)
    {
        camera = VideoSystem::instance()->createGenICamVideoSource(camera_ids);
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
            const std::string device = mHardwareTriggerPath->path().toStdString();
            struct stat info;

            if(ok)
            {
                ok = (0 == stat(device.c_str(), &info));
                err = "Trigger device does not exist!";
            }

            if(ok)
            {
                ok = ((info.st_mode & S_IFMT) == S_IFCHR);
                err = "Path is not a valid trigger device!";
            }

            if(ok)
            {
                camera->setHardwareTrigger(device);
            }
        }
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

void NewRecordingDialog::triggerChanged()
{
    mHardwareTriggerPath->setEnabled( mHardwareTrigger->isChecked() );
}

