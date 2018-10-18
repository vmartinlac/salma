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
    QObject::connect(btnselectpath, SIGNAL(clicked()), this, SLOT(selectOutputPath()));

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output path", pathwidget);

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
        newoutputpath = (mPath->text());
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

void CameraCalibrationParametersWidget::selectOutputPath()
{
    QString ret = QFileDialog::getSaveFileName( this, "Select output file", mPath->text(), "JSON file (*.json)" );

    if(ret.isEmpty() == false)
    {
        mPath->setText(ret);
    }
}

