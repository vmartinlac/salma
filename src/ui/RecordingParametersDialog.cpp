#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include "VimbaCamera.h"
#include "RecordingParametersDialog.h"

RecordingParametersDialog::RecordingParametersDialog(RecordingParameters* params, QWidget* parent) : QDialog(parent)
{
    mParameters = params;

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

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera", mCameraList);
    form->addRow("Output directory", pathwidget);

    QPushButton* okbutton = new QPushButton("OK");
    QPushButton* cancelbutton = new QPushButton("Cancel");

    QObject::connect(okbutton, SIGNAL(clicked()), this, SLOT(accept()));
    QObject::connect(cancelbutton, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout* buttonslay = new QHBoxLayout();
    buttonslay->addWidget(okbutton);
    buttonslay->addWidget(cancelbutton);

    QVBoxLayout* mainlay = new QVBoxLayout();
    mainlay->addLayout(form);
    mainlay->addLayout(buttonslay);

    setLayout(mainlay);
    setWindowTitle("Parameters");
}

void RecordingParametersDialog::accept()
{
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
        mParameters->beginWrite();
        mParameters->data().camera.swap(newcamera);
        mParameters->data().output_directory = newoutputdirectory;
        mParameters->endWrite();
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", error_message);
    }
}

void RecordingParametersDialog::selectOutputDirectory()
{
    QString ret = QFileDialog::getExistingDirectory(this, "Select output directory", mPath->text());

    if(ret.isEmpty() == false)
    {
        mPath->setText(ret);
    }
}

int RecordingParametersDialog::exec()
{
    mParameters->beginRead();
    mPath->setText( mParameters->data().output_directory.path() );
    mParameters->endRead();
    return QDialog::exec();
}

