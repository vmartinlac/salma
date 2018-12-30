#include <QToolBar>
#include <QMessageBox>
#include <QSplitter>
#include <QVBoxLayout>
#include "CameraCalibrationPanel.h"
#include "Project.h"
#include "VideoSystem.h"
#include "NewCameraCalibrationDialog.h"
#include "OperationDialog.h"
#include "CameraCalibrationModel.h"

CameraCalibrationPanel::CameraCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    mView->setModel(mProject->cameraCalibrationModel());

    connect(mView, SIGNAL(clicked(const QModelIndex&)), this, SLOT(onSelect(const QModelIndex&)));

    mText->setReadOnly(true);

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
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
            OperationDialog* opdlg = new OperationDialog(mProject, op, this);
            opdlg->exec();
            delete opdlg;

            mProject->cameraCalibrationModel()->refresh();
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least one camera!");
    }
}

void CameraCalibrationPanel::onRename()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void CameraCalibrationPanel::onDelete()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void CameraCalibrationPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int camera_id;
    bool ok = true;

    if(ok)
    {
        camera_id = mProject->cameraCalibrationModel()->indexToId(ind);
        ok = (camera_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeCamera(camera_id, text);
    }

    if(ok)
    {
        mText->setText(text);
    }
    else
    {
        mText->setText(QString());
    }
}

