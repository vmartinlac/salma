#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include "VideoSystem.h"
#include "StereoRecordingParametersWidget.h"
#include "StereoRecordingOperation.h"

StereoRecordingParametersWidget::StereoRecordingParametersWidget(QWidget* parent)
{
    mCameraList = new QComboBox();
    VideoSystem* vs = VideoSystem::instance();
    for(int i=0; i<vs->getNumberOfAvtCameras(); i++)
    {
        mCameraList->addItem(QString(vs->getNameOfAvtCamera(i).c_str()), i);
    }

    mPath = new QLineEdit();
    QPushButton* btnselectpath = new QPushButton("Select");
    QHBoxLayout* pathlay = new QHBoxLayout();
    pathlay->setContentsMargins(0, 0, 0, 0);
    pathlay->addWidget(mPath);
    pathlay->addWidget(btnselectpath);
    QWidget* pathwidget = new QWidget();
    pathwidget->setLayout(pathlay);
    QObject::connect(btnselectpath, SIGNAL(clicked()), this, SLOT(selectOutputDirectory()));

    mMaxFrameRate = new QSpinBox();
    mMaxFrameRate->setMinimum(1);
    mMaxFrameRate->setMaximum(1000);
    mMaxFrameRate->setValue(25);

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output directory", pathwidget);
    form->addRow("Max frame rate", mMaxFrameRate);

    setLayout(form);
}

OperationPtr StereoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QDir newoutputdirectory;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        QVariant data = mCameraList->currentData();

        VideoSystem* vs = VideoSystem::instance();

        if(data.isValid())
        {
            int id = data.toInt();
            if( 0 <= id && id < vs->getNumberOfAvtCameras() )
            {
                newcamera = vs->createMonoAvtVideoSource(id);
            }
        }
        
        ok = bool(newcamera);
        error_message = "Please select a camera!";
    }

    if(ok)
    {
        newoutputdirectory = QDir(mPath->text());
        ok = newoutputdirectory.mkpath(".");
        error_message = "Failed to create output directory!";
    }

    if(ok)
    {
        StereoRecordingOperation* op = new StereoRecordingOperation();
        ret.reset(op);

        op->mCamera.swap(newcamera);
        op->mOutputDirectory = newoutputdirectory;
    }
    else
    {
        QMessageBox::critical(this, "Error", error_message);
    }

    return ret;
}

QString StereoRecordingParametersWidget::name()
{
    return "Stereo-recording";
}

void StereoRecordingParametersWidget::selectOutputDirectory()
{
    QString ret = QFileDialog::getExistingDirectory(this, "Select output directory", mPath->text());

    if(ret.isEmpty() == false)
    {
        mPath->setText(ret);
    }
}

