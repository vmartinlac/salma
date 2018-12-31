#include <QFormLayout>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "NewMonoRecordingDialog.h"
#include "VideoSystem.h"
#include "RecordingOperation.h"

NewMonoRecordingDialog::NewMonoRecordingDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();
    mCamera = new CameraList();
    mFrameRate = new FrameRateWidget();
    mVisualizationOnly = new QCheckBox();

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);
    form->addRow("Camera:", mCamera);
    form->addRow("Max frame rate:", mFrameRate);
    form->addRow("Visualization only:", mVisualizationOnly);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New Mono Recording");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

NewMonoRecordingDialog::~NewMonoRecordingDialog()
{
}

void NewMonoRecordingDialog::accept()
{
    QString name;
    QDir directory;
    int camera_id = -1;
    VideoSourcePtr camera;
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
        visualization_only = mVisualizationOnly->isChecked();
    }

    if(ok)
    {
        if(visualization_only)
        {
            directory = QDir::temp();
        }
        else
        {
            ok = project()->createRecordingDirectory(directory);
            err = "Could not create new directory! Please check access rights!";
        }
    }

    if(ok)
    {
        RecordingOperation* myop = new RecordingOperation();
        myop->mRecordingName = name.toStdString();
        myop->mCamera = camera;
        myop->mDirectory = std::move(directory);
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

