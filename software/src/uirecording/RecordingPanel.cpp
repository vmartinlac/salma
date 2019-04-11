#include <QListWidget>
#include <QToolButton>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "RecordingPanel.h"
#include "Project.h"
#include "VideoSystem.h"
#include "NewRecordingDialog.h"
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

    QAction* aNewMono = new QAction("Mono",this);
    QAction* aNewStereo = new QAction("Stereo",this);
    QAction* aNewArbitrary = new QAction("Other",this);

    QToolButton* bNew = new QToolButton();
    bNew->setText("New");
    bNew->setPopupMode(QToolButton::InstantPopup);
    bNew->addAction(aNewMono);
    bNew->addAction(aNewStereo);
    bNew->addAction(aNewArbitrary);

    QToolBar* tb = new QToolBar();
    tb->addWidget(bNew);
    //QAction* aNewMono = tb->addAction("New mono");
    //QAction* aNewStereo = tb->addAction("New stereo");
    QAction* aPlay = tb->addAction("Play");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNewMono, SIGNAL(triggered()), this, SLOT(onNewMonoRecording()));
    connect(aNewStereo, SIGNAL(triggered()), this, SLOT(onNewStereoRecording()));
    connect(aNewArbitrary, SIGNAL(triggered()), this, SLOT(onNewArbitraryRecording()));
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
    onNewRecording(1);
}

void RecordingPanel::onNewStereoRecording()
{
    onNewRecording(2);
}

void RecordingPanel::onNewArbitraryRecording()
{
    bool ok = true;
    const int num_cameras = QInputDialog::getInt(this, "Number of cameras", "Number of cameras?", 1, 1, 10, 1, &ok);

    if(ok)
    {
        onNewRecording(num_cameras);
    }
}

void RecordingPanel::onNewRecording(int num_cameras)
{
    if( VideoSystem::instance()->getNumberOfGenICamCameras() >= num_cameras )
    {
        OperationPtr op;

        NewRecordingDialog* dlg = new NewRecordingDialog(num_cameras, mProject, this);
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
        QMessageBox::critical(this, "Error", "You need at least " + QString::number(num_cameras) + " camera(s)!");
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
        ok = mProject->loadRecording(recording_id, header) && bool(header);

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
    bool is_mutable = false;
    int recording_id = -1;
    bool ok = true;

    if(ok)
    {
        recording_id = mProject->recordingModel()->indexToId(mView->currentIndex());
        ok = (recording_id >= 0);
    }

    if(ok)
    {
        ok = mProject->isRecordingMutable(recording_id, is_mutable);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Error while checking whether the recording is referenced!");
        }
    }

    if(ok)
    {
        if(is_mutable)
        {
            const auto ret = QMessageBox::question(this, "Confirmation", "Do you really want to delete selected recording?", QMessageBox::Yes|QMessageBox::No);

            if(ret == QMessageBox::Yes)
            {
                const bool removal_status = mProject->removeRecording(recording_id);

                if(removal_status == false)
                {
                    QMessageBox::critical(this, "Error", "Could not remove selected recording!");
                }
            }
        }
        else
        {
            QMessageBox::critical(this, "Error", "This recording is referenced by another item and cannot be deleted!");
        }
    }
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

