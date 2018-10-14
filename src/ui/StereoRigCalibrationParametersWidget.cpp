#include <QFileInfo>
#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QFormLayout>
#include "VimbaCamera.h"
#include "StereoRigCalibrationParametersWidget.h"
#include "StereoRigCalibrationOperation.h"

StereoRigCalibrationParametersWidget::StereoRigCalibrationParametersWidget(QWidget* parent) : OperationParametersWidget(parent)
{
    mCameraList = new QComboBox();

    VimbaCameraManager& vimba = VimbaCameraManager::instance();

    for(int i=0; i<vimba.getNumCameras(); i++)
    {
        mCameraList->addItem(QString(vimba.getCamera(i)->getHumanName().c_str()), i);
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

OperationPtr StereoRigCalibrationParametersWidget::getOperation()
{
    OperationPtr ret;

    CameraPtr newcamera;
    QString newoutputpath;

    bool ok = true;
    const char* error_message;

    if(ok)
    {
        QVariant data = mCameraList->currentData();

        VimbaCameraManager& vimba = VimbaCameraManager::instance();

        if(data.isValid())
        {
            int id = data.toInt();
            if( 0 <= id && id < vimba.getNumCameras() )
            {
                newcamera = vimba.getCamera(id);
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
        StereoRigCalibrationOperation* op = new StereoRigCalibrationOperation();
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

QString StereoRigCalibrationParametersWidget::name()
{
    return "Stereo rig calibration";
}

void StereoRigCalibrationParametersWidget::selectOutputPath()
{
    QString ret = QFileDialog::getSaveFileName( this, "Select output file", mPath->text(), "JSON file (*.json)" );

    if(ret.isEmpty() == false)
    {
        mPath->setText(ret);
    }
}

