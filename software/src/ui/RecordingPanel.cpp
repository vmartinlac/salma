#include <QListWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "RecordingPanel.h"
#include "Project.h"
#include "VideoSystem.h"
#include "NewMonoRecordingDialog.h"
#include "NewStereoRecordingDialog.h"
#include "OperationDialog.h"
#include "RecordingPlayerDialog.h"

RecordingPanel::RecordingPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mText = new QTextEdit();
    mView = new QListView();

    connect(project, SIGNAL(recordingModelChanged()), this, SLOT(onModelChanged()));

    mView->setModel(mProject->recordingModel());

    connect(mView, SIGNAL(clicked(const QModelIndex&)), this, SLOT(onSelect(const QModelIndex&)));
    connect(mView, SIGNAL(doubleClicked(const QModelIndex&)), this, SLOT(onPlayRecording()));

    mText->setReadOnly(true);

    QToolBar* tb = new QToolBar();
    QAction* aNewMono = tb->addAction("New mono");
    QAction* aNewStereo = tb->addAction("New stereo");
    QAction* aPlay = tb->addAction("Play");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNewMono, SIGNAL(triggered()), this, SLOT(onNewMonoRecording()));
    connect(aNewStereo, SIGNAL(triggered()), this, SLOT(onNewStereoRecording()));
    connect(aPlay, SIGNAL(triggered()), this, SLOT(onPlayRecording()));
    connect(aRename, SIGNAL(triggered()), this, SLOT(onRenameRecording()));
    connect(aDelete, SIGNAL(triggered()), this, SLOT(onDeleteRecording()));

    QSplitter* splitter = new QSplitter();
    splitter->addWidget(mView);
    splitter->addWidget(mText);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
}

void RecordingPanel::onNewMonoRecording()
{
    if( VideoSystem::instance()->getNumberOfGenICamCameras() >= 1 )
    {
        OperationPtr op;

        NewMonoRecordingDialog* dlg = new NewMonoRecordingDialog(mProject, this);
        dlg->exec();
        op = dlg->getOperation();
        delete dlg;

        if(op)
        {
            OperationDialog* opdlg = new OperationDialog(mProject, op, this);
            opdlg->exec();
            delete opdlg;

            mProject->recordingModel()->refresh();
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least one camera!");
    }
}

void RecordingPanel::onNewStereoRecording()
{
    if( VideoSystem::instance()->getNumberOfGenICamCameras() >= 2 )
    {
        OperationPtr op;

        NewStereoRecordingDialog* dlg = new NewStereoRecordingDialog(mProject, this);
        dlg->exec();
        op = dlg->getOperation();
        delete dlg;

        if(op)
        {
            OperationDialog* opdlg = new OperationDialog(mProject, op, this);
            opdlg->exec();
            delete opdlg;

            mProject->recordingModel()->refresh();
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "You need at least two cameras!");
    }
}

void RecordingPanel::onPlayRecording()
{
    int recording_id = -1;
    bool ok = true;
    RecordingHeaderPtr header;

    if(ok)
    {
        recording_id = mProject->recordingModel()->indexToId(mView->currentIndex());
        ok = (recording_id >= 0);
    }

    if(ok)
    {
        mProject->loadRecording(recording_id, header);
        ok = bool(header);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not load recording!");
        }
    }

    if(ok)
    {
        RecordingPlayerDialog* dlg = new RecordingPlayerDialog(header, this);
        dlg->exec();
        delete dlg;
    }
}

void RecordingPanel::onRenameRecording()
{
    QString text;
    int recording_id = -1;
    bool ok = true;

    if(ok)
    {
        recording_id = mProject->recordingModel()->indexToId(mView->currentIndex());
        ok = (recording_id >= 0);
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
        ok = mProject->renameRecording(recording_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }
    }
}

void RecordingPanel::onDeleteRecording()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void RecordingPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int recording_id;
    bool ok = true;

    if(ok)
    {
        recording_id = mProject->recordingModel()->indexToId(ind);
        ok = (recording_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeRecording(recording_id, text);
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

void RecordingPanel::onModelChanged()
{
    mText->clear();
}

