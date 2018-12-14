#include <QPushButton>
#include <QSettings>
#include <QMessageBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include "VideoSystem.h"
#include "MonoRecordingParametersWidget.h"
#include "MonoRecordingOperation.h"

MonoRecordingParametersWidget::MonoRecordingParametersWidget(QWidget* parent)
{
    mPath = new PathWidget(PathWidget::GET_EXISTING_DIRECTORY);
    mPath->setPath(".");

    mCameraList = new CameraList();

    mMaxFrameRate = new QSpinBox();
    mMaxFrameRate->setValue(1000);
    mMaxFrameRate->setMinimum(1);
    mMaxFrameRate->setMaximum(1000);

    mVisualizationOnly = new QCheckBox();

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output directory", mPath);
    form->addRow("Max framerate", mMaxFrameRate);
    form->addRow("Visualization only", mVisualizationOnly);

    setLayout(form);

    QSettings s;
    s.beginGroup("mono_recording_parameters");
    mCameraList->setSelectedCamera( s.value("camera", QString() ).toString().toStdString() );
    mPath->setPath( s.value("output_path", ".").toString() );
    mVisualizationOnly->setChecked( s.value("visualization_only", false).toBool() );
    mMaxFrameRate->setValue( s.value("max_framerate", 1000).toInt() );
    s.endGroup();
}

OperationPtr MonoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QDir newoutputdirectory;
    int camera_id = -1;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        camera_id = mCameraList->getCameraId();

        if( camera_id >= 0 )
        {
            newcamera = VideoSystem::instance()->createVideoSourceGenICamMono(camera_id);
        }
        
        ok = bool(newcamera);
        error_message = "Please select a camera!";
    }

    if(ok)
    {
        ok = mPath->path().isEmpty() == false;
        error_message = "Please set output directory!";
    }

    if(ok)
    {
        newoutputdirectory = QDir(mPath->path());
        ok = newoutputdirectory.mkpath(".");
        error_message = "Failed to create output directory!";
    }

    if(ok)
    {
        MonoRecordingOperation* op = new MonoRecordingOperation();
        ret.reset(op);

        op->mCamera.swap(newcamera);
        op->mOutputDirectory = newoutputdirectory;
        op->mVisualizationOnly = mVisualizationOnly->isChecked();
        op->mMaxFrameRate = mMaxFrameRate->value();

        QSettings s;
        s.beginGroup("mono_recording_parameters");
        s.setValue("camera", VideoSystem::instance()->getNameOfGenICamCamera(camera_id).c_str());
        s.setValue("output_path", mPath->path());
        s.setValue("max_framerate", mMaxFrameRate->value());
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

QString MonoRecordingParametersWidget::name()
{
    return "Mono-recording";
}
