#include <QToolBar>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include "AboutDialog.h"
#include "OperationDialog.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
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
        QObject::connect(mActionStart, SIGNAL(triggered()), this, SLOT(startOperation()));
        QObject::connect(mActionStop, SIGNAL(triggered()), this, SLOT(stopOperation()));
        QObject::connect(mActionAbout, SIGNAL(triggered()), this, SLOT(about()));

        mActionStop->setEnabled(false);

        mActionConfigure->setShortcut(QKeySequence("Alt+C"));
        mActionStart->setShortcut(QKeySequence("F5"));
        mActionStop->setShortcut(QKeySequence("Maj+F5"));
        mActionAbout->setShortcut(QKeySequence("F1"));
    }

    // set up central widgets.

    {
        mVideoWidget = new VideoWidget();

        mStatsWidget = new StatsWidget();

        QScrollArea* scroll = new QScrollArea();
        scroll->setAlignment(Qt::AlignCenter);
        scroll->setWidget(mVideoWidget);

        QSplitter* splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mStatsWidget);

        setCentralWidget(splitter);
    }

    // set up engine and other data structures.

    {
        mThread = new OperationThread( mVideoWidget->getPort(), mStatsWidget->getPort(), this );

        QObject::connect(mThread, SIGNAL(started()), this, SLOT(operationStarted()));
        QObject::connect(mThread, SIGNAL(finished()), this, SLOT(operationStopped()));
    }

    setWindowTitle("Calibration & Recording");
}

MainWindow::~MainWindow()
{
    if( mThread->isRunning() )
    {
        mThread->requestInterruption();
        mThread->wait();
    }
}

void MainWindow::configure()
{
    if( mThread->isRunning() == false )
    {
        OperationDialog* dlg = new OperationDialog(this);

        const int ret = dlg->exec();

        if(ret == QDialog::Accepted)
        {
            mThread->setOperation( dlg->getOperation() );
        }

        delete dlg;
    }
}

void MainWindow::startOperation()
{
    mThread->start();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void MainWindow::stopOperation()
{
    mThread->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void MainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void MainWindow::operationStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void MainWindow::operationStopped()
{
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

