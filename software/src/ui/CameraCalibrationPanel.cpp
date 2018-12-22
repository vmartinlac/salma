#include <QToolBar>
#include <QMessageBox>
#include <QSplitter>
#include <QVBoxLayout>
#include "CameraCalibrationPanel.h"
#include "ImportCameraCalibrationDialog.h"
#include "Project.h"
#include "VideoSystem.h"

CameraCalibrationPanel::CameraCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    mView->setModel(mProject->cameraCalibrationModel());

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
    QAction* aImport = tb->addAction("Import");
    QAction* aExport = tb->addAction("Export");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
    connect(aImport, SIGNAL(triggered()), this, SLOT(onImport()));
    connect(aExport, SIGNAL(triggered()), this, SLOT(onExport()));
    connect(aRename, SIGNAL(triggered()), this, SLOT(onRename()));
    connect(aDelete, SIGNAL(triggered()), this, SLOT(onDelete()));

    QSplitter* splitter = new QSplitter();
    splitter->addWidget(mView);
    splitter->addWidget(mText);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
}

void CameraCalibrationPanel::onNew()
{
    //if( VideoSystem::instance()->getNumberOfGenICamCameras() > 0 )
    if(true)
    {
        //NewCameraCalibrationDialog* dlg = new NewCameraCalibrationDialog(mProject, this);
        //dlg->exec();
        //delete dlg;
    }
    else
    {
        QMessageBox::critical(this, "Error", "Not camera was detected!");
    }
}

void CameraCalibrationPanel::onImport()
{
    ImportCameraCalibrationDialog* dlg = new ImportCameraCalibrationDialog(mProject, this);
    dlg->exec();
    delete dlg;
}

void CameraCalibrationPanel::onExport()
{
}

void CameraCalibrationPanel::onRename()
{
}

void CameraCalibrationPanel::onDelete()
{
}

