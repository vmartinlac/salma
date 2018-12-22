#include <QFormLayout>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "CameraCalibrationData.h"
#include "ImportCameraCalibrationDialog.h"
#include "Project.h"

ImportCameraCalibrationDialog::ImportCameraCalibrationDialog(Project* project, QWidget* parent) : QDialog(parent)
{
    mProject = project;
    mPath = new PathWidget(PathWidget::GET_OPEN_FILENAME);
    mName = new QLineEdit();

    QFormLayout* form = new QFormLayout();
    form->addRow("Path", mPath);
    form->addRow("Name", mName);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setWindowTitle("Import camera calibration");
    setLayout(vlay);

    connect( btnok, SIGNAL(clicked()), this, SLOT(accept()) );
    connect( btncancel, SIGNAL(clicked()), this, SLOT(reject()) );
}

void ImportCameraCalibrationDialog::accept()
{
    CameraCalibrationData cam;
    bool ok = true;
    const char* err = "";

    if(ok)
    {
        ok = cam.loadFromFile(mPath->path().toStdString());
        err = "Could not load calibration data!";
    }

    if(ok)
    {
        int camera_id;
        mProject->saveCamera(cam, camera_id);
        err = "Could not save calibration data into database!";
    }

    if(ok)
    {
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

