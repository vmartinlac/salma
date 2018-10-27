#include <QFileInfo>
#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QFormLayout>
#include "VideoSystem.h"
#include "CameraCalibrationParametersWidget.h"
#include "CameraCalibrationOperation.h"

CameraCalibrationParametersWidget::CameraCalibrationParametersWidget(QWidget* parent) : OperationParametersWidget(parent)
{
    mCameraList = new CameraList();

    mPath = new PathWidget(PathWidget::GET_SAVE_FILENAME);

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output JSON file", mPath);

    setLayout(form);
}

OperationPtr CameraCalibrationParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QString newoutputpath;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        int camera_id = mCameraList->getCameraId();

        if(camera_id >= 0)
        {
            newcamera = VideoSystem::instance()->createMonoAvtVideoSource(camera_id);
        }
        
        ok = bool(newcamera);
        error_message = "Please select a camera!";
    }

    if(ok)
    {
        newoutputpath = mPath->path();
        ok = (newoutputpath.isEmpty() == false);
        error_message = "Please set an output filename!";
    }

    if(ok)
    {
        CameraCalibrationOperation* op = new CameraCalibrationOperation();
        ret.reset(op);

        op->mOutputPath = newoutputpath.toStdString();
        op->mCamera.swap(newcamera);
    }
    else
    {
        QMessageBox::critical(this, "Error", error_message);
    }

    return ret;
}

QString CameraCalibrationParametersWidget::name()
{
    return "Camera calibration";
}

