#include <QDir>
#include <QSettings>
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

    QSettings s;
    s.beginGroup("stereo_recording_parameters");
    mLeftCamera->setSelectedCamera( s.value("left_camera", QString() ).toString().toStdString() );
    mRightCamera->setSelectedCamera( s.value("right_camera", QString() ).toString().toStdString() );
    mOutputPath->setPath( s.value("output_path", QString()).toString() );
    mVisualizationOnly->setChecked( s.value("visualization_only", false).toBool() );
    s.endGroup();
}

OperationPtr StereoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QDir newoutputdirectory;
    QString outputpath;

    int left = -1;
    int right = -1;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        left = mLeftCamera->getCameraId();
        right = mRightCamera->getCameraId();

        if( left >= 0 && right >= 0 )
        {
            newcamera = VideoSystem::instance()->createVideoSourceGenICamStereo(left, right);
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

        QSettings s;
        s.beginGroup("stereo_recording_parameters");
        s.setValue("left_camera", VideoSystem::instance()->getNameOfGenICamCamera(left).c_str());
        s.setValue("right_camera", VideoSystem::instance()->getNameOfGenICamCamera(right).c_str());
        s.setValue("output_path", mOutputPath->path());
        s.setValue("visualization_only", mVisualizationOnly->isChecked());
        s.endGroup();
        s.sync();
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

