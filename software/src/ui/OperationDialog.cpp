#include <QToolBar>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include "OperationDialog.h"

OperationDialog::OperationDialog(OperationPtr op, QWidget* parent) : QDialog(parent)
{
    mOperation = op;

    // set up the toolbar.

    QToolBar* tb = nullptr;
    QSplitter* splitter = nullptr;

    {
        tb = new QToolBar("ToolBar");

        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        mActionStart = tb->addAction("Start");
        mActionStop = tb->addAction("Stop");

        mActionQuit->setIcon(QIcon::fromTheme("application-exit"));
        mActionConfigure->setIcon(QIcon::fromTheme("document-properties"));
        mActionStart->setIcon(QIcon::fromTheme("media-playback-start"));
        mActionStop->setIcon(QIcon::fromTheme("media-playback-stop"));
        mActionAbout->setIcon(QIcon::fromTheme("help-about"));

        /*
        QObject::connect(mActionQuit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
        QObject::connect(mActionConfigure, SIGNAL(triggered()), this, SLOT(configure()));
        QObject::connect(mActionStart, SIGNAL(triggered()), this, SLOT(startOperation()));
        QObject::connect(mActionStop, SIGNAL(triggered()), this, SLOT(stopOperation()));
        QObject::connect(mActionAbout, SIGNAL(triggered()), this, SLOT(about()));
        */

        mActionStop->setEnabled(false);

        mActionQuit->setShortcut(QKeySequence("Esc"));
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

        splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mStatsWidget);
    }

    // set up engine and other data structures.

    {
        mThread = new OperationThread( mVideoWidget->getPort(), mStatsWidget->getPort(), this );

        QObject::connect(mThread, SIGNAL(started()), this, SLOT(operationStarted()));
        QObject::connect(mThread, SIGNAL(finished()), this, SLOT(operationStopped()));
    }

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
    setWindowTitle(mOperation->getName());
}

OperationDialog::~OperationDialog()
{
    if( mThread->isRunning() )
    {
        mThread->requestInterruption();
        mThread->wait();
    }
}

/*
void OperationDialog::configure()
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

void OperationDialog::startOperation()
{
    mThread->start();
    mActionConfigure->setEnabled(false);
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void OperationDialog::stopOperation()
{
    mThread->requestInterruption();
    mActionConfigure->setEnabled(false);
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void OperationDialog::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void OperationDialog::operationStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
    mActionConfigure->setEnabled(false);
}

void OperationDialog::operationStopped()
{
    mActionConfigure->setEnabled(true);
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

*/
