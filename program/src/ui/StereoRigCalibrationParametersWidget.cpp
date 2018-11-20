#include <QFileInfo>
#include <QSettings>
#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QFormLayout>
#include "VideoSystem.h"
#include "StereoRigCalibrationParametersWidget.h"
#include "StereoRigCalibrationOperation.h"

StereoRigCalibrationParametersWidget::StereoRigCalibrationParametersWidget(QWidget* parent) : OperationParametersWidget(parent)
{
    mLeftCamera = new CameraList();

    mPathToLeftCalibrationData = new PathWidget(PathWidget::GET_OPEN_FILENAME);

    mRightCamera = new CameraList();

    mPathToRightCalibrationData = new PathWidget(PathWidget::GET_OPEN_FILENAME);

    mTargetParameters = new TargetParametersWidget();

    mNumViews = new QSpinBox();
    mNumViews->setMinimum(3);
    mNumViews->setMaximum(1000);

    mOutputPath = new PathWidget(PathWidget::GET_SAVE_FILENAME);

    QFormLayout* form = new QFormLayout();
    form->addRow("Left camera", mLeftCamera);
    form->addRow("Left camera calibration data", mPathToLeftCalibrationData);
    form->addRow("Right camera", mRightCamera);
    form->addRow("Right camera calibration data", mPathToRightCalibrationData);
    form->addRow("Target cell length", mTargetParameters);
    form->addRow("Number of views", mNumViews);
    form->addRow("Output JSON file", mOutputPath);

    setLayout(form);

    QSettings s;
    s.beginGroup("stereo_rig_calibration_parameters");
    mLeftCamera->setSelectedCamera( s.value("left_camera", QString() ).toString().toStdString() );
    mRightCamera->setSelectedCamera( s.value("right_camera", QString() ).toString().toStdString() );
    mPathToLeftCalibrationData->setPath( s.value("left_camera_calibration_file", QString()).toString() );
    mPathToRightCalibrationData->setPath( s.value("right_camera_calibration_file", QString()).toString() );
    mTargetParameters->setCellLength( s.value("target_cell_length", 1.0).toDouble() );
    mNumViews->setValue( s.value("num_views", 20).toInt() );
    mOutputPath->setPath( s.value("output_file", QString()).toString() );
    s.endGroup();
}

OperationPtr StereoRigCalibrationParametersWidget::getOperation()
{
    OperationPtr ret;

    CameraCalibrationData left_camera_parameters;
    CameraCalibrationData right_camera_parameters;
    VideoSourcePtr newcamera;
    QString newoutputpath;
    int left_id = -1;
    int right_id = -1;

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
        left_id = mLeftCamera->getCameraId();
        right_id = mRightCamera->getCameraId();

        newcamera = VideoSystem::instance()->createVideoSourceGenICamStereo(left_id, right_id);
        
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
        op->mTargetCellLength = mTargetParameters->getCellLength();
        op->mNumberOfPosesForCalibration = mNumViews->value();
        op->mCamera.swap(newcamera);

        QSettings s;
        s.beginGroup("stereo_rig_calibration_parameters");
        s.setValue("left_camera", VideoSystem::instance()->getNameOfGenICamCamera(left_id).c_str());
        s.setValue("right_camera", VideoSystem::instance()->getNameOfGenICamCamera(right_id).c_str());
        s.setValue("left_camera_calibration_file", mPathToLeftCalibrationData->path());
        s.setValue("right_camera_calibration_file", mPathToRightCalibrationData->path());
        s.setValue("target_cell_length", mTargetParameters->getCellLength());
        s.setValue("num_views", mNumViews->value());
        s.setValue("output_file", mOutputPath->path());
        s.endGroup();
        s.sync();
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

