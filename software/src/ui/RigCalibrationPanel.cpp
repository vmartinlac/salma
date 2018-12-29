#include <QListWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QMessageBox>
#include <QVBoxLayout>
#include "RigCalibrationPanel.h"
#include "OperationDialog.h"
#include "VideoSystem.h"
#include "NewRigCalibrationDialog.h"

RigCalibrationPanel::RigCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    mView->setModel(mProject->rigCalibrationModel());

    mText->setReadOnly(true);

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New stereo");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
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
            OperationDialog* opdlg = new OperationDialog(mProject, op, this);
            opdlg->exec();
            delete opdlg;

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

void RigCalibrationPanel::onDelete()
{
    QMessageBox::critical(this, "Error", "Not implemented!");
}

