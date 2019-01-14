#include <QToolBar>
#include <QInputDialog>
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

    connect(project, SIGNAL(cameraCalibrationModelChanged()), this, SLOT(onModelChanged()));

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
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least one camera!");
    }
}

void CameraCalibrationPanel::onRename()
{
    QString text;
    int camera_id;
    bool ok = true;

    if(ok)
    {
        camera_id = mProject->cameraCalibrationModel()->indexToId(mView->currentIndex());
        ok = (camera_id >= 0);
    }

    if(ok)
    {
        bool accepted;
        text = QInputDialog::getText(this, "Rename", "New name?", QLineEdit::Normal, "", &accepted);

        ok = accepted && (text.isEmpty() == false);

        if(accepted == true && ok == false)
        {
            QMessageBox::critical(this, "Error", "Incorrect name!");
        }
    }

    if(ok)
    {
        ok = mProject->renameCamera(camera_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }

        //mProject->cameraCalibrationModel()->refresh();
    }
}

void CameraCalibrationPanel::onDelete()
{
    bool is_mutable = false;
    int camera_id = -1;
    bool ok = true;

    if(ok)
    {
        camera_id = mProject->cameraCalibrationModel()->indexToId(mView->currentIndex());
        ok = (camera_id >= 0);
    }

    if(ok)
    {
        ok = mProject->isCameraMutable(camera_id, is_mutable);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Error while checking whether the camera is referenced!");
        }
    }

    if(ok)
    {
        if(is_mutable)
        {
            const auto ret = QMessageBox::question(this, "Confirmation", "Do you really want to delete selected camera?", QMessageBox::Yes|QMessageBox::No);

            if(ret == QMessageBox::Yes)
            {
                const bool removal_status = mProject->removeCamera(camera_id);

                if(removal_status == false)
                {
                    QMessageBox::critical(this, "Error", "Could not remove selected camera!");
                }
            }
        }
        else
        {
            QMessageBox::critical(this, "Error", "This camera is referenced by another item and cannot be deleted!");
        }
    }
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

void CameraCalibrationPanel::onModelChanged()
{
    mText->clear();
}

