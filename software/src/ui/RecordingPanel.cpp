#include <QListWidget>
#include <QMessageBox>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "RecordingPanel.h"
#include "Project.h"
#include "VideoSystem.h"
#include "NewStereoRecordingDialog.h"
#include "OperationDialog.h"

RecordingPanel::RecordingPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mText = new QTextEdit();
    mView = new QListView();

    mView->setModel(mProject->recordingModel());

    mText->setReadOnly(true);

    QToolBar* tb = new QToolBar();
    QAction* aNewMono = tb->addAction("New mono recording");
    QAction* aNewStereo = tb->addAction("New stereo recording");
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
    QMessageBox::critical(this, "Error", "Not implemented");
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
        QMessageBox::critical(this, "Error", "You need at least one camera!");
    }
}

void RecordingPanel::onPlayRecording()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void RecordingPanel::onRenameRecording()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void RecordingPanel::onDeleteRecording()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

