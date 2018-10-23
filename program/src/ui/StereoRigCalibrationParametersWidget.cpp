#include <QFileInfo>
#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QFormLayout>
#include "VideoSystem.h"
#include "StereoRigCalibrationParametersWidget.h"
#include "StereoRigCalibrationOperation.h"

StereoRigCalibrationParametersWidget::StereoRigCalibrationParametersWidget(QWidget* parent) : OperationParametersWidget(parent)
{
    mLeftCamera = new QComboBox();
    mRightCamera = new QComboBox();
    mPathToLeftCalibrationData = new PathWidget(PathWidget::GET_OPEN_FILENAME);
    mPathToRightCalibrationData = new PathWidget(PathWidget::GET_OPEN_FILENAME);

    VideoSystem* vs = VideoSystem::instance();

    for(int i=0; i<vs->getNumberOfAvtCameras(); i++)
    {
        QString name = QString(vs->getNameOfAvtCamera(i).c_str());
        mLeftCamera->addItem(name, i);
        mRightCamera->addItem(name, i);
    }

    mOutputPath = new PathWidget(PathWidget::GET_SAVE_FILENAME);

    QFormLayout* form = new QFormLayout();
    form->addRow("Left camera", mLeftCamera);
    form->addRow("Left camera calibration data", mPathToLeftCalibrationData);
    form->addRow("Right camera", mRightCamera);
    form->addRow("Right camera calibration data", mPathToRightCalibrationData);
    form->addRow("Output JSON file", mOutputPath);

    setLayout(form);
}

OperationPtr StereoRigCalibrationParametersWidget::getOperation()
{
    OperationPtr ret;

    CameraCalibrationData left_camera_parameters;
    CameraCalibrationData right_camera_parameters;
    VideoSourcePtr newcamera;
    QString newoutputpath;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        ok = left_camera_parameters.loadFromFile(mPathToLeftCalibrationData->path().toStdString());
        error_message = "Left camera calibration data is incorrect!";
    }

    if(ok)
    {
        ok = right_camera_parameters.loadFromFile(mPathToRightCalibrationData->path().toStdString());
        error_message = "Right camera calibration data is incorrect!";
    }

    if(ok)
    {
        QVariant left_data = mLeftCamera->currentData();
        QVariant right_data = mRightCamera->currentData();

        VideoSystem* vs = VideoSystem::instance();

        if( left_data.isValid() && right_data.isValid() )
        {
            int left_id = left_data.toInt();
            int right_id = right_data.toInt();
            if( 0 <= left_id && left_id < vs->getNumberOfAvtCameras() && 0 <= right_id && right_id < vs->getNumberOfAvtCameras() )
            {
                newcamera = vs->createStereoAvtVideoSource(left_id, right_id);
            }
        }
        
        ok = bool(newcamera);
        error_message = "Incorrect camera!";
    }

    if(ok)
    {
        newoutputpath = (mOutputPath->path());
        ok = (newoutputpath.isEmpty() == false);
        error_message = "Please set an output filename!";
    }

    if(ok)
    {
        StereoRigCalibrationOperation* op = new StereoRigCalibrationOperation();
        ret.reset(op);

        op->mOutputPath = newoutputpath.toStdString();
        op->mLeftCalibrationData = left_camera_parameters;
        op->mRightCalibrationData = right_camera_parameters;
        op->mCamera.swap(newcamera);
    }
    else
    {
        QMessageBox::critical(this, "Error", error_message);
    }

    return ret;
}

QString StereoRigCalibrationParametersWidget::name()
{
    return "Stereo rig calibration";
}

