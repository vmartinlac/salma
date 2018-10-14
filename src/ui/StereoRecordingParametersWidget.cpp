#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include "VimbaCamera.h"
#include "StereoRecordingParametersWidget.h"
#include "StereoRecordingOperation.h"

StereoRecordingParametersWidget::StereoRecordingParametersWidget(QWidget* parent)
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
    QObject::connect(btnselectpath, SIGNAL(clicked()), this, SLOT(selectOutputDirectory()));

    mMaxFrameRate = new QSpinBox();
    mMaxFrameRate->setMinimum(1);
    mMaxFrameRate->setMaximum(1000);
    mMaxFrameRate->setValue(25);

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output directory", pathwidget);
    form->addRow("Max frame rate", mMaxFrameRate);

    setLayout(form);
}

OperationPtr StereoRecordingParametersWidget::getOperation()
{
    OperationPtr ret;

    CameraPtr newcamera;
    QDir newoutputdirectory;

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
        newoutputdirectory = QDir(mPath->text());
        ok = newoutputdirectory.mkpath(".");
        error_message = "Failed to create output directory!";
    }

    if(ok)
    {
        StereoRecordingOperation* op = new StereoRecordingOperation();
        ret.reset(op);

        op->mCamera.swap(newcamera);
        op->mOutputDirectory = newoutputdirectory;
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

void StereoRecordingParametersWidget::selectOutputDirectory()
{
    QString ret = QFileDialog::getExistingDirectory(this, "Select output directory", mPath->text());

    if(ret.isEmpty() == false)
    {
        mPath->setText(ret);
    }
}

