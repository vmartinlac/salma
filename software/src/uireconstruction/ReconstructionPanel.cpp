#include <QListWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "ReconstructionPanel.h"
#include "Project.h"
#include "OperationDialog.h"
#include "NewReconstructionDialog.h"
#include "ReconstructionInspectorDialog.h"

ReconstructionPanel::ReconstructionPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mText = new QTextEdit();
    mView = new QListView();

    connect(project, SIGNAL(reconstructionModelChanged()), this, SLOT(onModelChanged()));

    mView->setModel(mProject->reconstructionModel());

    connect(mView, SIGNAL(clicked(const QModelIndex&)), this, SLOT(onSelect(const QModelIndex&)));
    connect(mView, SIGNAL(doubleClicked(const QModelIndex&)), this, SLOT(onShow()));

    mText->setReadOnly(true);
    
    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
    QAction* aShow = tb->addAction("Show");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
    connect(aShow, SIGNAL(triggered()), this, SLOT(onShow()));
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

void ReconstructionPanel::onNew()
{
    bool ok = true;
    const char* err = "";

    RecordingList recordings;
    CalibrationList calibrations;

    mProject->listRecordings(recordings);
    mProject->listCalibrations(calibrations);

    if(ok)
    {
        ok = ( recordings.empty() == false );
        err = "You need at least one recording!";
    }

    if(ok)
    {
        ok = ( calibrations.empty() == false );
        err = "You need at least one calibration!";
    }

    if(ok)
    {
        OperationPtr op;

        NewReconstructionDialog* dlg = new NewReconstructionDialog(mProject, this);
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

    if(ok == false)
    {
        QMessageBox::critical(this, "Error", err);
    }
}

void ReconstructionPanel::onShow()
{
    int reconstruction_id = -1;
    bool ok = true;
    SLAMReconstructionPtr reconstruction;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(mView->currentIndex());
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        ok = mProject->loadReconstruction(reconstruction_id, reconstruction) && bool(reconstruction);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not load reconstruction!");
        }
    }

    if(ok)
    {
        ok = ( reconstruction->segments.empty() == false );

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "This reconstruction is empty!");
        }
    }

    if(ok)
    {
        ReconstructionInspectorDialog* dlg = new ReconstructionInspectorDialog(reconstruction, this);
        dlg->exec();
        delete dlg;
    }
}

void ReconstructionPanel::onRename()
{
    QString text;
    int reconstruction_id = -1;
    bool ok = true;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(mView->currentIndex());
        ok = (reconstruction_id >= 0);
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
        ok = mProject->renameReconstruction(reconstruction_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }
    }
}

void ReconstructionPanel::onDelete()
{
    bool is_mutable = false;
    int reconstruction_id = -1;
    bool ok = true;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(mView->currentIndex());
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        ok = mProject->isReconstructionMutable(reconstruction_id, is_mutable);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Error while checking whether the reconstruction is referenced!");
        }
    }

    if(ok)
    {
        if(is_mutable)
        {
            const auto ret = QMessageBox::question(this, "Confirmation", "Do you really want to delete selected reconstruction?", QMessageBox::Yes|QMessageBox::No);

            if(ret == QMessageBox::Yes)
            {
                const bool removal_status = mProject->removeReconstruction(reconstruction_id);

                if(removal_status == false)
                {
                    QMessageBox::critical(this, "Error", "Could not remove selected reconstruction!");
                }
            }
        }
        else
        {
            QMessageBox::critical(this, "Error", "This reconstruction is referenced by another item and cannot be deleted!");
        }
    }
}

void ReconstructionPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int reconstruction_id;
    bool ok = true;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(ind);
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeReconstruction(reconstruction_id, text);
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

void ReconstructionPanel::onModelChanged()
{
    mText->clear();
}

