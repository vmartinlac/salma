#include <QSettings>
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

    mTargetParameters = new TargetParametersWidget();

    mPath = new PathWidget(PathWidget::GET_SAVE_FILENAME);

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Target cell length", mTargetParameters);
    form->addRow("Output JSON file", mPath);

    setLayout(form);

    QSettings s;
    s.beginGroup("camera_calibration_parameters");
    mCameraList->setSelectedCamera( s.value("camera", QString() ).toString().toStdString() );
    mTargetParameters->setCellLength( s.value("target_cell_length", 1.0).toDouble() );
    mPath->setPath( s.value("output_file", "camera.json").toString() );
    s.endGroup();
}

OperationPtr CameraCalibrationParametersWidget::getOperation()
{
    OperationPtr ret;

    VideoSourcePtr newcamera;
    QString newoutputpath;
    int camera_id = -1;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        camera_id = mCameraList->getCameraId();

        if(camera_id >= 0)
        {
            newcamera = VideoSystem::instance()->createVideoSourceGenICamMono(camera_id);
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
        op->mTargetCellLength = mTargetParameters->getCellLength();
        op->mCamera.swap(newcamera);

        QSettings s;
        s.beginGroup("camera_calibration_parameters");
        s.setValue("camera", VideoSystem::instance()->getNameOfGenICamCamera(camera_id).c_str());
        s.setValue("target_cell_length", mTargetParameters->getCellLength());
        s.setValue("output_file", mPath->path());
        s.endGroup();
        s.sync();
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

