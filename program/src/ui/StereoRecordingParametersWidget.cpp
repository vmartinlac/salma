#include <QDir>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include "VideoSystem.h"
#include "StereoRecordingParametersWidget.h"
#include "StereoRecordingOperation.h"

StereoRecordingParametersWidget::StereoRecordingParametersWidget(QWidget* parent)
{
    mLeftCamera = new CameraList();

    mRightCamera = new CameraList();

    mOutputPath = new PathWidget(PathWidget::GET_EXISTING_DIRECTORY);
    mOutputPath->setPath(".");

    mVisualizationOnly = new QCheckBox();

    /*
    mMaxFrameRate = new QSpinBox();
    mMaxFrameRate->setMinimum(1);
    mMaxFrameRate->setMaximum(1000);
    mMaxFrameRate->setValue(25);
    */

    QFormLayout* form = new QFormLayout();
    form->addRow("Left camera", mLeftCamera);
    form->addRow("Right camera", mRightCamera);
    form->addRow("Output directory", mOutputPath);
    form->addRow("Visualization only", mVisualizationOnly);

    setLayout(form);
}

OperationPtr StereoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QDir newoutputdirectory;
    QString outputpath;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        const int left = mLeftCamera->getCameraId();
        const int right = mRightCamera->getCameraId();

        if( left >= 0 && right >= 0 )
        {
            newcamera = VideoSystem::instance()->createStereoAvtVideoSource(left, right);
        }
        
        ok = bool(newcamera);
        error_message = "Incorrect camera!";
    }

    if(ok)
    {
        outputpath = mOutputPath->path();
        ok = (outputpath.isEmpty() == false);
        error_message = "Please set an output directory!";
    }

    if(ok)
    {
        newoutputdirectory = QDir(outputpath);
        ok = newoutputdirectory.mkpath(".");
        error_message = "Failed to create output directory!";
    }

    if(ok)
    {
        StereoRecordingOperation* op = new StereoRecordingOperation();
        ret.reset(op);

        op->mCamera.swap(newcamera);
        op->mOutputDirectory = newoutputdirectory;
        op->mVisualizationOnly = mVisualizationOnly->isChecked();
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

