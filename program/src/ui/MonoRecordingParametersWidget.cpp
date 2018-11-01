#include <QPushButton>
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

    mVisualizationOnly = new QCheckBox();

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output directory", mPath);
    form->addRow("Visualization only", mVisualizationOnly);

    setLayout(form);
}

OperationPtr MonoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QDir newoutputdirectory;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        const int camera_id = mCameraList->getCameraId();

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
