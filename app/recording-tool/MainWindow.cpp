#include <QToolBar>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include "ParametersDialog.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    // set up engine and other data structures.

    {
        mParameters = new Parameters(this);
        mOutput = new Output(this);
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

        mVideoWidget = new VideoWidget(mOutput);

        mInformationWidget = new InformationWidget(mOutput);

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

void MainWindow::configure()
{
    ParametersDialog* dlg = new ParametersDialog(mParameters, this);

    int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        ;
    }

    delete dlg;
}

void MainWindow::startRecording()
{
    mEngine->start();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void MainWindow::stopRecording()
{
    mEngine->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Tool to record a video. Written by Victor Martin Lac 2018.");
}

void MainWindow::engineStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void MainWindow::engineStopped()
{
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

