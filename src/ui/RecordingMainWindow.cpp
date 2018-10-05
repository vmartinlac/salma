#include <QToolBar>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include "AboutDialog.h"
#include "RecordingParametersDialog.h"
#include "RecordingMainWindow.h"

RecordingMainWindow::RecordingMainWindow(QWidget* parent) : QMainWindow(parent)
{
    // set up engine and other data structures.

    {
        mParameters = new RecordingParameters(this);
        mOutput = new RecordingOutput(this);
        mEngine = new RecordingThread(mParameters, mOutput, this);

        QObject::connect(mEngine, SIGNAL(started()), this, SLOT(engineStarted()));
        QObject::connect(mEngine, SIGNAL(finished()), this, SLOT(engineStopped()));
    }

    // set up the toolbar.

    {
        QToolBar* tb = addToolBar("ToolBar");

        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        mActionConfigure = tb->addAction("Configure");
        mActionStart = tb->addAction("Start");
        mActionStop = tb->addAction("Stop");
        mActionAbout = tb->addAction("About");

        mActionConfigure->setIcon(QIcon::fromTheme("document-properties"));
        mActionStart->setIcon(QIcon::fromTheme("media-playback-start"));
        mActionStop->setIcon(QIcon::fromTheme("media-playback-stop"));
        mActionAbout->setIcon(QIcon::fromTheme("help-about"));

        mActionConfigure->setShortcut(QKeySequence("Alt+C"));

        QObject::connect(mActionConfigure, SIGNAL(triggered()), this, SLOT(configure()));
        QObject::connect(mActionStart, SIGNAL(triggered()), this, SLOT(startRecording()));
        QObject::connect(mActionStop, SIGNAL(triggered()), this, SLOT(stopRecording()));
        QObject::connect(mActionAbout, SIGNAL(triggered()), this, SLOT(about()));

        mActionStop->setEnabled(false);
    }

    // set up central widgets.

    {

        mVideoWidget = new RecordingVideoWidget(mOutput);

        mInformationWidget = new RecordingInformationWidget(mOutput);

        QScrollArea* scroll = new QScrollArea();
        scroll->setAlignment(Qt::AlignCenter);
        scroll->setWidget(mVideoWidget);

        QSplitter* splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mInformationWidget);

        setCentralWidget(splitter);
    }

    setWindowTitle("Video Recorder");
}

void RecordingMainWindow::configure()
{
    RecordingParametersDialog* dlg = new RecordingParametersDialog(mParameters, this);

    int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        ;
    }

    delete dlg;
}

void RecordingMainWindow::startRecording()
{
    mEngine->start();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void RecordingMainWindow::stopRecording()
{
    mEngine->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void RecordingMainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void RecordingMainWindow::engineStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void RecordingMainWindow::engineStopped()
{
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

