#include <QToolBar>
#include <QMessageBox>
#include <QSplitter>
#include <QVBoxLayout>
#include "CameraCalibrationPanel.h"
#include "Project.h"
#include "VideoSystem.h"
#include "NewCameraCalibrationDialog.h"
#include "OperationDialog.h"

CameraCalibrationPanel::CameraCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    mView->setModel(mProject->cameraCalibrationModel());

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
    QAction* aRename = tb->addAction("Rename");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
    connect(aRename, SIGNAL(triggered()), this, SLOT(onRename()));

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
    if( VideoSystem::instance()->getNumberOfGenICamCameras() > 0 )
    {
        OperationPtr op;

        NewCameraCalibrationDialog* dlg = new NewCameraCalibrationDialog(mProject, this);
        dlg->exec();
        op = dlg->getOperation();
        delete dlg;

        if(op)
        {
            OperationDialog* opdlg = new OperationDialog(op, this);
            opdlg->exec();
            delete opdlg;
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "No camera was detected!");
    }
}

void CameraCalibrationPanel::onRename()
{
}

