#include <QListWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QMessageBox>
#include <QVBoxLayout>
#include "RigCalibrationPanel.h"
#include "VideoSystem.h"
#include "NewRigCalibrationDialog.h"

RigCalibrationPanel::RigCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    mView->setModel(mProject->rigCalibrationModel());

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New stereo calibration");
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

void RigCalibrationPanel::onNew()
{
    if( VideoSystem::instance()->getNumberOfGenICamCameras() >= 2 )
    {
        OperationPtr op;

        NewRigCalibrationDialog* dlg = new NewRigCalibrationDialog(mProject, this);
        dlg->exec();
        op = dlg->getOperation();
        delete dlg;

        if(op)
        {
            mProject->rigCalibrationModel()->refresh();
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least two cameras!");
    }
}

void RigCalibrationPanel::onRename()
{
    QMessageBox::critical(this, "Error", "Not implemented!");
}

