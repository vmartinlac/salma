#include <QListWidget>
#include <QInputDialog>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QMessageBox>
#include <QVBoxLayout>
#include "CalibrationPanel.h"
#include "OperationDialog.h"
#include "VideoSystem.h"
#include "NewManualCalibrationDialog.h"
#include "ManualCalibrationDialog.h"

CalibrationPanel::CalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QListView();
    mText = new QTextEdit();

    connect(project, SIGNAL(calibrationModelChanged()), this, SLOT(onModelChanged()));

    mView->setModel(mProject->calibrationModel());

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

void CalibrationPanel::onNew()
{
    if(mProject->isOpen() == false)
    {
        QMessageBox::critical(this, "Error", "No project is open!");
    }
    else
    {
        RecordingList recs;
        mProject->listRecordings(recs);

        if(recs.empty())
        {
            QMessageBox::critical(this, "Error", "You need at least one recording!");
        }
        else
        {
            NewManualCalibrationDialog* dlg = new NewManualCalibrationDialog(mProject, this);

            dlg->exec();

            ManualCalibrationParametersPtr params = dlg->getParameters();

            delete dlg;

            if(params)
            {
                ManualCalibrationDialog* dlg2 = new ManualCalibrationDialog(mProject, params, this);
                dlg2->exec();
                delete dlg2;
            }
        }
    }
}

void CalibrationPanel::onRename()
{
    QString text;
    int rig_id;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->calibrationModel()->indexToId(mView->currentIndex());
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
        ok = mProject->renameCalibration(rig_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }

        //mProject->calibrationModel()->refresh();
    }
}

void CalibrationPanel::onDelete()
{
    bool is_mutable = false;
    int rig_id = -1;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->calibrationModel()->indexToId(mView->currentIndex());
        ok = (rig_id >= 0);
    }

    if(ok)
    {
        ok = mProject->isCalibrationMutable(rig_id, is_mutable);

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
                const bool removal_status = mProject->removeCalibration(rig_id);

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

void CalibrationPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int rig_id;
    bool ok = true;

    if(ok)
    {
        rig_id = mProject->calibrationModel()->indexToId(ind);
        ok = (rig_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeCalibration(rig_id, text);
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

void CalibrationPanel::onModelChanged()
{
    mText->clear();
}

