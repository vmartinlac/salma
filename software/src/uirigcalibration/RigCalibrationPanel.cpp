#include <QListWidget>
#include <QInputDialog>
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

    connect(project, SIGNAL(rigCalibrationModelChanged()), this, SLOT(onModelChanged()));

    mView->setModel(mProject->rigCalibrationModel());

    connect(mView, SIGNAL(clicked(const QModelIndex&)), this, SLOT(onSelect(const QModelIndex&)));

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

            //mProject->rigCalibrationModel()->refresh();
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least two cameras!");
    }
}

void RigCalibrationPanel::onRename()
{
    QString text;
    int rig_id;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->rigCalibrationModel()->indexToId(mView->currentIndex());
        ok = (rig_id >= 0);
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
        ok = mProject->renameRig(rig_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }

        //mProject->rigCalibrationModel()->refresh();
    }
}

void RigCalibrationPanel::onDelete()
{
    bool is_mutable = false;
    int rig_id = -1;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->rigCalibrationModel()->indexToId(mView->currentIndex());
        ok = (rig_id >= 0);
    }

    if(ok)
    {
        ok = mProject->isRigMutable(rig_id, is_mutable);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Error while checking whether the rig is referenced!");
        }
    }

    if(ok)
    {
        if(is_mutable)
        {
            const auto ret = QMessageBox::question(this, "Confirmation", "Do you really want to delete selected rig?", QMessageBox::Yes|QMessageBox::No);

            if(ret == QMessageBox::Yes)
            {
                const bool removal_status = mProject->removeRig(rig_id);

                if(removal_status == false)
                {
                    QMessageBox::critical(this, "Error", "Could not remove selected rig!");
                }
            }
        }
        else
        {
            QMessageBox::critical(this, "Error", "This rig is referenced by another item and cannot be deleted!");
        }
    }
}

void RigCalibrationPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int rig_id;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->rigCalibrationModel()->indexToId(ind);
        ok = (rig_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeRig(rig_id, text);
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

void RigCalibrationPanel::onModelChanged()
{
    mText->clear();
}

